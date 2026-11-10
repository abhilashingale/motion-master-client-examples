/**
 * Homes every joint at its current position, one after the other, announcing each joint by its
 * assigned name first so whoever is at the robot can see which one is being homed.
 *
 * Homing method 37 ("current position") is used: Homing method (0x6098:00) is set to 37 and the
 * homing operation is started. Method 37 only defines the present position as the home position;
 * it is not meant to move the joint.
 *
 * For each joint, in bus order:
 *
 *   1. Print the joint's assigned name (0x20F2:00, e.g. "104 head_yaw") and pause --pause-ms so it
 *      can be seen.
 *   2. Modes of operation (0x6060:00) = 6 (Homing), Homing method (0x6098:00) = 37.
 *   3. Controlword 0x06 -> 0x07 -> 0x0F (Shutdown, Switch on, Enable operation).
 *   4. Controlword 0x1F: start homing; wait for Homing attained (statusword bit 12).
 *   5. Read the Actual position (0x6064:00) and report it in degrees; it should be close to 0.
 *   6. Controlword 0x00 (Switched on disabled) and the previous mode of operation restored —
 *      also when the homing failed.
 *
 * It stops at the first joint that fails and does not touch the remaining ones. Ctrl-C stops
 * after the joint that is currently being homed.
 *
 * This ENERGIZES each drive (Operation enabled, Homing mode) one at a time. Run it yourself at the
 * robot with the approved E-stop/STO within reach, and only once every joint is where it should be
 * when it becomes home. Nothing runs without typing "yes". --dry-run lists the joints in the order
 * they would be homed and does nothing else.
 *
 * Usage:
 *   # List the joints and the order, nothing is written
 *   npx ts-node src/humanoid_bringup/home-joints.ts --dry-run
 *
 *   # Home every labelled joint, one after the other
 *   npx ts-node src/humanoid_bringup/home-joints.ts
 *
 *   # One joint only, or a longer pause between joints
 *   npx ts-node src/humanoid_bringup/home-joints.ts --device-number 104
 *   npx ts-node src/humanoid_bringup/home-joints.ts --pause-ms 5000
 */

import { createInterface } from "readline";

import { Option, program } from "commander";
import { lastValueFrom } from "rxjs";

import { client } from "../init-client";

const DEVICE_LABEL_INDEX = 0x20f2;
const DEVICE_LABEL_SUBINDEX = 0;
const CONTROLWORD_INDEX = 0x6040;
const STATUSWORD_INDEX = 0x6041;
const MODES_OF_OPERATION_INDEX = 0x6060;
const POSITION_ACTUAL_INDEX = 0x6064;
const HOME_OFFSET_INDEX = 0x607c;
const HOMING_METHOD_INDEX = 0x6098;

const HOMING_MODE = 6;
const HOMING_METHOD_CURRENT_POSITION = 37;
const SW_FAULT = 1 << 3;
const SW_OPERATION_ENABLED = 1 << 2;
const SW_HOMING_ATTAINED = 1 << 12;
const SW_HOMING_ERROR = 1 << 13;
const CW_SHUTDOWN = 0x06;
const CW_SWITCH_ON = 0x07;
const CW_ENABLE_OPERATION = 0x0f;
const CW_START_HOMING = 0x1f;
const CW_DISABLE_VOLTAGE = 0x00;

const ASSUMED_POSITION_ENCODER_BITS = 20;
const STATE_STEP_DELAY_MS = 200;
const HOMING_TIMEOUT_MS = 10000;

const sleep = (ms: number) => new Promise((resolve) => setTimeout(resolve, ms));

program
  .addOption(new Option("--device-number <value>", "only home the joint whose 0x20F2:00 label starts with this number, e.g. 104"))
  .addOption(
    new Option("--pause-ms <ms>", "pause after announcing each joint, before homing it")
      .default(2000)
      .argParser((v) => {
        const n = parseInt(v, 10);
        if (!Number.isInteger(n) || n < 0) throw new Error(`Invalid --pause-ms "${v}".`);
        return n;
      }),
  )
  .addOption(
    new Option("--home-tolerance-deg <deg>", "largest |Actual position| after homing that counts as close to 0")
      .default(0.5)
      .argParser(Number),
  )
  .addOption(new Option("--dry-run", "list the joints in the order they would be homed, nothing else").default(false));

program.allowExcessArguments(false);
program.showHelpAfterError("(run with --help to see the available options)");
program.parse();

const { deviceNumber, pauseMs, homeToleranceDeg, dryRun, requestTimeout = 10000 } = program.opts<{
  deviceNumber?: string;
  pauseMs: number;
  homeToleranceDeg: number;
  dryRun: boolean;
  requestTimeout?: number;
}>();

let stopRequested = false;
process.on("SIGINT", () => {
  if (stopRequested) process.exit(130);
  stopRequested = true;
  console.warn("\n[Ctrl-C] Finishing the current joint, then stopping. Press Ctrl-C again to quit immediately.");
});

function confirm(promptText: string): Promise<boolean> {
  const rl = createInterface({ input: process.stdin, output: process.stdout });
  return new Promise((resolve) => {
    rl.question(promptText, (answer) => {
      rl.close();
      resolve(answer.trim().toLowerCase() === "yes");
    });
  });
}

function banner(lines: string[]): void {
  const inner = Math.max(...lines.map((l) => l.length)) + 4;
  const bar = "!".repeat(inner + 4);
  const blank = `!!${" ".repeat(inner)}!!`;
  const body = lines.map((l) => `!!  ${l.padEnd(inner - 2)}!!`);
  console.warn(`\n${[bar, bar, blank, ...body, blank, bar, bar].join("\n")}\n`);
}

interface LiveDevice {
  deviceRef: string | number;
  label: string;
  number: string;
}

/** Labelled devices in bus order. */
async function scanLiveDevices(): Promise<LiveDevice[]> {
  const devices = await lastValueFrom(client.request.getDevices(requestTimeout));
  console.log(`Discovered ${devices.length} device(s) on the network.`);
  const live: LiveDevice[] = [];
  for (const device of [...devices].sort((a, b) => (a.position ?? 0) - (b.position ?? 0))) {
    const deviceRef = device.serialNumber ?? device.deviceAddress;
    try {
      const label = String(
        await client.request.upload<string>(deviceRef, DEVICE_LABEL_INDEX, DEVICE_LABEL_SUBINDEX, false, requestTimeout),
      );
      const number = label.trim().match(/^(\d+)\b/)?.[1];
      if (number) live.push({ deviceRef, label, number });
    } catch {
      // No 0x20F2:00 — not a labelled joint drive.
    }
  }
  return live;
}

const upload = (d: LiveDevice, index: number, subindex: number) =>
  client.request.upload<number>(d.deviceRef, index, subindex, false, requestTimeout);
const download = (d: LiveDevice, index: number, subindex: number, value: number) =>
  client.request.download(d.deviceRef, index, subindex, value, undefined, requestTimeout);

/** Homing method 37 on one drive; always returns it to Switched on disabled. */
async function homeAtCurrentPosition(d: LiveDevice): Promise<{ ok: boolean; status: string }> {
  let previousMode: number | undefined;
  try {
    const before = await upload(d, STATUSWORD_INDEX, 0);
    if (before & SW_FAULT) return { ok: false, status: `NOT STARTED (fault, statusword 0x${before.toString(16)})` };
    if (before & SW_OPERATION_ENABLED) {
      return { ok: false, status: "NOT STARTED (drive is already operation enabled — something else is controlling it)" };
    }

    previousMode = await upload(d, MODES_OF_OPERATION_INDEX, 0);
    const homeOffset = await upload(d, HOME_OFFSET_INDEX, 0);

    await download(d, MODES_OF_OPERATION_INDEX, 0, HOMING_MODE);
    await download(d, HOMING_METHOD_INDEX, 0, HOMING_METHOD_CURRENT_POSITION);

    for (const cw of [CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OPERATION]) {
      await download(d, CONTROLWORD_INDEX, 0, cw);
      await sleep(STATE_STEP_DELAY_MS);
    }
    const afterEnable = await upload(d, STATUSWORD_INDEX, 0);
    if (afterEnable & SW_FAULT || !(afterEnable & SW_OPERATION_ENABLED)) {
      return { ok: false, status: `NOT STARTED (statusword 0x${afterEnable.toString(16)}: not operation enabled)` };
    }

    await download(d, CONTROLWORD_INDEX, 0, CW_START_HOMING);
    const deadline = Date.now() + HOMING_TIMEOUT_MS;
    let statusword = 0;
    while (Date.now() < deadline) {
      statusword = await upload(d, STATUSWORD_INDEX, 0);
      if (statusword & (SW_HOMING_ATTAINED | SW_HOMING_ERROR | SW_FAULT)) break;
      await sleep(50);
    }
    if (statusword & (SW_HOMING_ERROR | SW_FAULT)) {
      return { ok: false, status: `HOMING ERROR (statusword 0x${statusword.toString(16)})` };
    }
    if (!(statusword & SW_HOMING_ATTAINED)) {
      return { ok: false, status: `HOMING TIMEOUT (statusword 0x${statusword.toString(16)})` };
    }

    const counts = await upload(d, POSITION_ACTUAL_INDEX, 0);
    const degrees = (counts / 2 ** ASSUMED_POSITION_ENCODER_BITS) * 360;
    const close = Math.abs(degrees) <= homeToleranceDeg;
    return {
      ok: close,
      status:
        `${close ? "HOMED" : "HOMED, POSITION NOT NEAR 0"} (actual position ${degrees.toFixed(3)}°, ` +
        `home offset 0x607C:00 = ${homeOffset}${homeOffset !== 0 ? " — a non-zero home offset shifts the result" : ""})`,
    };
  } catch (err: unknown) {
    return { ok: false, status: `FAILED (${err instanceof Error ? err.message : err})` };
  } finally {
    try {
      await download(d, CONTROLWORD_INDEX, 0, CW_DISABLE_VOLTAGE);
      if (previousMode !== undefined) await download(d, MODES_OF_OPERATION_INDEX, 0, previousMode);
    } catch {
      console.error(`  [ERROR] could not return ${d.label} to Switched on disabled — check the drive.`);
    }
  }
}

async function main(): Promise<void> {
  console.log(`\n=== Home joints at the current position (method ${HOMING_METHOD_CURRENT_POSITION})${dryRun ? " (DRY RUN)" : ""} ===`);

  let joints = await scanLiveDevices();
  if (deviceNumber) joints = joints.filter((d) => d.number === deviceNumber);
  if (joints.length === 0) {
    throw new Error(deviceNumber ? `No labelled device with number ${deviceNumber}.` : "No labelled devices found.");
  }

  console.log(`\n${joints.length} joint(s), in the order they will be homed:\n`);
  console.table(joints.map((d, i) => ({ Order: i + 1, Joint: d.label })));

  if (dryRun) {
    console.log("Dry run — nothing written, no joint homed.\n");
    return;
  }

  banner([
    `HOMING (method ${HOMING_METHOD_CURRENT_POSITION}) will ENERGIZE ${joints.length} drive(s), one after the other`,
    "Each joint is homed where it is now: its current position becomes home",
    "It is not meant to move the joint - keep clear and keep the approved E-stop/STO within reach",
  ]);
  if (!(await confirm(`Type "yes" to home these ${joints.length} joint(s) one by one, anything else to abort: `))) {
    console.log("\nAborted — no joint homed.\n");
    return;
  }

  const results: { Order: number; Joint: string; Status: string }[] = [];
  for (const [i, joint] of joints.entries()) {
    if (stopRequested) {
      console.log("\nStopped on request — the remaining joints were not homed.");
      break;
    }

    console.log(`\n>>> [${i + 1}/${joints.length}] HOMING NOW: ${joint.label} <<<`);
    await sleep(pauseMs);

    const { ok, status } = await homeAtCurrentPosition(joint);
    console.log(`    ${status}`);
    results.push({ Order: i + 1, Joint: joint.label, Status: status });

    if (!ok) {
      console.error(`\n[STOP] ${joint.label} did not complete — the remaining joints were not homed.`);
      break;
    }
  }

  console.log("\n=== Homing summary ===");
  console.table(results);
  if (results.length < joints.length) {
    console.log(`${joints.length - results.length} joint(s) not homed.\n`);
  }
}

client
  .whenReady()
  .then(main)
  .catch((err: unknown) => {
    console.error(`[FATAL] ${err instanceof Error ? err.message : err}`);
    process.exitCode = 1;
  })
  .finally(() => client.closeSockets());

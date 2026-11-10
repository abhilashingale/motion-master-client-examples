/**
 * Centres the Encoder 2 single-turn range on the joint's home position, so the joint gets its
 * full range of motion without the single-turn value overflowing.
 *
 * The goal is to set Encoder 2 single turn offset (0x2112:06) so that Encoder 2 feedback
 * Adjusted position (0x2113:02) reads exactly 2^19 — the middle of a 2^20 single-turn range —
 * with the joint at its home position.
 *
 * Move every joint to its home position BEFORE running this. The script cannot do that for you.
 *
 * Per drive:
 *
 *   1. Read --samples (5-10) samples of Encoder 2 feedback Raw position (0x2113:01) and Adjusted
 *      position (0x2113:02), and refuse to continue if the joint is moving (spread above
 *      --max-spread counts).
 *   2. Calculate the single turn offset from the raw position:
 *
 *        IF raw < resolution / 2   offset = raw + 2^19
 *        ELSE                      offset = raw - 2^19
 *
 *      i.e. the raw value moved half a turn, wrapped into [0, resolution).
 *   3. Write 0x2112:06 and set Restore home position when loading configuration (0x2005:02) to 1
 *      (YES), save all parameters (0x1010: Store parameters), then read Adjusted position back.
 *      The drive counts as a SUCCESS only when it reads 2^19 within --tolerance counts. On
 *      anything else the previous offset and restore flag are written back and saved.
 * This only reads and writes parameters; it never energizes a drive. Homing at the current
 * position (method 37) is a separate step: `home-joints.ts`.
 *
 * Drives are matched like the other scripts here: by the number at the start of the 0x20F2:00
 * label. Without --device-number every labelled drive is processed.
 *
 * Usage:
 *   # Read-only: table of adjusted position now, calculated ST offset and expected adjusted position.
 *   # Nothing is written.
 *   npx ts-node src/humanoid_bringup/set-encoder-st-offset.ts --dry-run
 *
 *   # Set and save the offsets (asks for "yes" first)
 *   npx ts-node src/humanoid_bringup/set-encoder-st-offset.ts
 *
 *   # One joint only
 *   npx ts-node src/humanoid_bringup/set-encoder-st-offset.ts --device-number 104
 */

import { createInterface } from "readline";

import { Option, program } from "commander";
import { lastValueFrom } from "rxjs";

import { client } from "../init-client";

const DEVICE_LABEL_INDEX = 0x20f2;
const DEVICE_LABEL_SUBINDEX = 0;
const ENCODER2_ST_OFFSET_INDEX = 0x2112;
const ENCODER2_ST_OFFSET_SUBINDEX = 6;
const ENCODER2_RAW_INDEX = 0x2113;
const ENCODER2_RAW_SUBINDEX = 1;
const ENCODER2_ADJUSTED_SUBINDEX = 2;
const RESTORE_HOME_POSITION_INDEX = 0x2005;
const RESTORE_HOME_POSITION_SUBINDEX = 2;
const STATUSWORD_INDEX = 0x6041;

const SW_OPERATION_ENABLED = 1 << 2;

const SAMPLE_INTERVAL_MS = 100;

const sleep = (ms: number) => new Promise((resolve) => setTimeout(resolve, ms));

program
  .addOption(
    new Option("--samples <n>", "samples of the encoder position to read per drive (5-10)")
      .default(10)
      .argParser((v) => {
        const n = parseInt(v, 10);
        if (!Number.isInteger(n) || n < 5 || n > 10) throw new Error(`Invalid --samples "${v}". Expected 5-10.`);
        return n;
      }),
  )
  .addOption(
    new Option("--resolution-bits <n>", "Encoder 2 single-turn resolution in bits (the middle is 2^(n-1))")
      .default(20)
      .argParser((v) => {
        const n = parseInt(v, 10);
        if (!Number.isInteger(n) || n < 2 || n > 31) throw new Error(`Invalid --resolution-bits "${v}".`);
        return n;
      }),
  )
  .addOption(
    new Option("--max-spread <counts>", "refuse a drive whose raw position varies by more than this over the samples")
      .default(16)
      .argParser(Number),
  )
  .addOption(
    new Option("--tolerance <counts>", "how far from the middle the read-back adjusted position may be to count as success")
      .default(4)
      .argParser(Number),
  )
  .addOption(new Option("--device-number <value>", "only process the joint whose 0x20F2:00 label starts with this number, e.g. 104"))
  .addOption(new Option("--dry-run", "read and calculate only, write nothing").default(false));

program.allowExcessArguments(false);
program.showHelpAfterError("(run with --help to see the available options)");
program.parse();

const {
  samples,
  resolutionBits,
  maxSpread,
  tolerance,
  deviceNumber,
  dryRun,
  requestTimeout = 10000,
} = program.opts<{
  samples: number;
  resolutionBits: number;
  maxSpread: number;
  tolerance: number;
  deviceNumber?: string;
  dryRun: boolean;
  requestTimeout?: number;
}>();

const RESOLUTION = 2 ** resolutionBits;
const HALF = RESOLUTION / 2;

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

/** Wraps `n` into [0, RESOLUTION). */
const wrap = (n: number) => ((n % RESOLUTION) + RESOLUTION) % RESOLUTION;

/** Shortest signed distance from b to a on the circle of RESOLUTION counts. */
function circularDiff(a: number, b: number): number {
  const d = wrap(a - b);
  return d > HALF ? d - RESOLUTION : d;
}

/** The single turn offset that puts `raw` at the middle of the range. */
function calculateOffset(raw: number): number {
  return raw < HALF ? raw + HALF : raw - HALF;
}

/**
 * Mean of samples on the circle, so a joint sitting on the 0 / RESOLUTION seam does not average
 * to the middle of the range; also returns the largest distance from the mean.
 */
function circularMean(values: number[]): { mean: number; spread: number } {
  const reference = values[0];
  const meanDiff = values.reduce((sum, v) => sum + circularDiff(v, reference), 0) / values.length;
  const mean = wrap(Math.round(reference + meanDiff));
  const spread = Math.max(...values.map((v) => Math.abs(circularDiff(v, mean))));
  return { mean, spread };
}

interface LiveDevice {
  deviceRef: string | number;
  label: string;
  number: string;
}

async function scanLiveDevices(): Promise<LiveDevice[]> {
  const devices = await lastValueFrom(client.request.getDevices(requestTimeout));
  console.log(`Discovered ${devices.length} device(s) on the network.`);
  const live: LiveDevice[] = [];
  for (const device of devices) {
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

async function readSamples(d: LiveDevice, count: number) {
  const raw: number[] = [];
  const adjusted: number[] = [];
  for (let i = 0; i < count; i++) {
    raw.push(await upload(d, ENCODER2_RAW_INDEX, ENCODER2_RAW_SUBINDEX));
    adjusted.push(await upload(d, ENCODER2_RAW_INDEX, ENCODER2_ADJUSTED_SUBINDEX));
    if (i < count - 1) await sleep(SAMPLE_INTERVAL_MS);
  }
  return { raw: circularMean(raw), adjusted: circularMean(adjusted) };
}

interface Plan {
  device: LiveDevice;
  raw: number;
  adjustedBefore: number;
  currentOffset: number;
  restoreHomeBefore: number;
  newOffset: number;
  skip?: string;
}

async function plan(d: LiveDevice): Promise<Plan> {
  const blank = { device: d, raw: NaN, adjustedBefore: NaN, currentOffset: NaN, restoreHomeBefore: NaN, newOffset: NaN };
  try {
    const statusword = await upload(d, STATUSWORD_INDEX, 0);
    if (statusword & SW_OPERATION_ENABLED) {
      return { ...blank, skip: "OPERATION ENABLED (drive is holding the joint)" };
    }
    const currentOffset = await upload(d, ENCODER2_ST_OFFSET_INDEX, ENCODER2_ST_OFFSET_SUBINDEX);
    const restoreHomeBefore = await upload(d, RESTORE_HOME_POSITION_INDEX, RESTORE_HOME_POSITION_SUBINDEX);
    const { raw, adjusted } = await readSamples(d, samples);
    if (raw.spread > maxSpread) {
      return { ...blank, currentOffset, restoreHomeBefore, skip: `JOINT MOVING (raw spread ${raw.spread} > ${maxSpread})` };
    }
    return {
      device: d,
      raw: raw.mean,
      adjustedBefore: adjusted.mean,
      currentOffset,
      restoreHomeBefore,
      newOffset: calculateOffset(raw.mean),
    };
  } catch (err: unknown) {
    return { ...blank, skip: `READ FAILED (${err instanceof Error ? err.message : err})` };
  }
}

interface Outcome {
  device: LiveDevice;
  status: string;
  success: boolean;
}

/** Writes the offset and restore flag, saves, and reads the adjusted position back. Rolls back on failure. */
async function applyOffset(p: Plan): Promise<Outcome> {
  const d = p.device;
  const outcome = (status: string, success = false): Outcome => ({ device: d, status, success });
  try {
    await download(d, ENCODER2_ST_OFFSET_INDEX, ENCODER2_ST_OFFSET_SUBINDEX, p.newOffset);
    await download(d, RESTORE_HOME_POSITION_INDEX, RESTORE_HOME_POSITION_SUBINDEX, 1);
    await lastValueFrom(client.request.saveConfig(d.deviceRef));
  } catch (err: unknown) {
    return outcome(`WRITE/SAVE FAILED (${err instanceof Error ? err.message : err}) — check the drive before continuing`);
  }

  try {
    const { adjusted } = await readSamples(d, 5);
    const error = circularDiff(adjusted.mean, HALF);
    if (Math.abs(error) <= tolerance && adjusted.spread <= maxSpread) {
      return outcome(`SUCCESS (adjusted position ${adjusted.mean}, ${error >= 0 ? "+" : ""}${error} from the middle)`, true);
    }
    // Wrong result: put back what was there before.
    try {
      await download(d, ENCODER2_ST_OFFSET_INDEX, ENCODER2_ST_OFFSET_SUBINDEX, p.currentOffset);
      await download(d, RESTORE_HOME_POSITION_INDEX, RESTORE_HOME_POSITION_SUBINDEX, p.restoreHomeBefore);
      await lastValueFrom(client.request.saveConfig(d.deviceRef));
      return outcome(`FAILED, ROLLED BACK (adjusted position ${adjusted.mean}, ${error} from the middle)`);
    } catch (err: unknown) {
      return outcome(
        `FAILED AND ROLLBACK FAILED (${err instanceof Error ? err.message : err}) — restore 0x2112:06=${p.currentOffset}, 0x2005:02=${p.restoreHomeBefore} manually`,
      );
    }
  } catch (err: unknown) {
    return outcome(`VERIFY FAILED (${err instanceof Error ? err.message : err})`);
  }
}

async function main(): Promise<void> {
  console.log(`\n=== Set Encoder 2 single turn offset${dryRun ? " (DRY RUN)" : ""} ===`);
  banner([
    "Every joint must be at its HOME position before this step",
    `Sets 0x2112:06 so 0x2113:02 reads ${HALF} (middle of 2^${resolutionBits}) and SAVES it on the drive`,
  ]);

  let live = await scanLiveDevices();
  if (deviceNumber) live = live.filter((d) => d.number === deviceNumber);
  if (live.length === 0) {
    throw new Error(deviceNumber ? `No labelled device with number ${deviceNumber}.` : "No labelled devices found.");
  }
  console.log(`Reading ${samples} sample(s) from ${live.length} drive(s)...\n`);

  const plans: Plan[] = [];
  for (const d of live) plans.push(await plan(d));

  const show = (n: number) => (Number.isNaN(n) ? "-" : n);
  console.table(
    plans.map((p) => ({
      Joint: p.device.label,
      "Adjusted before (0x2113:02)": show(p.adjustedBefore),
      "Raw (0x2113:01)": show(p.raw),
      "ST offset now (0x2112:06)": show(p.currentOffset),
      "Calculated ST offset": show(p.newOffset),
      "Expected adjusted after": Number.isNaN(p.newOffset) ? "-" : wrap(p.raw - p.newOffset),
      Status: p.skip ?? (dryRun ? "WOULD SET" : "READY"),
    })),
  );

  // The calculation assumes adjusted = raw - offset; say so when the drive does not behave that way.
  for (const p of plans) {
    if (p.skip) continue;
    const expected = wrap(p.raw - p.currentOffset);
    if (Math.abs(circularDiff(expected, p.adjustedBefore)) > Math.max(tolerance, maxSpread)) {
      console.warn(
        `[WARN] ${p.device.label}: adjusted position ${p.adjustedBefore} is not raw - offset (${expected}). ` +
          `The result is verified after writing and rolled back if it is not ${HALF}.`,
      );
    }
  }

  const ready = plans.filter((p) => !p.skip);
  if (dryRun) {
    console.log(`\nDry run — nothing written.\n`);
    return;
  }
  if (ready.length === 0) {
    console.log("\nNothing to do.\n");
    return;
  }

  const confirmed = await confirm(
    `\nType "yes" to write and SAVE the offset and Restore home position = 1 on ${ready.length} drive(s), anything else to abort: `,
  );
  if (!confirmed) {
    console.log("\nAborted — nothing written.\n");
    return;
  }

  const outcomes: Outcome[] = [];
  for (const p of ready) outcomes.push(await applyOffset(p));
  console.log("\n=== Offset summary ===");
  console.table(outcomes.map((o) => ({ Joint: o.device.label, Status: o.status })));

  console.log("Next: home at the current position with home-joints.ts.\n");
}

client
  .whenReady()
  .then(main)
  .catch((err: unknown) => {
    console.error(`[FATAL] ${err instanceof Error ? err.message : err}`);
    process.exitCode = 1;
  })
  .finally(() => client.closeSockets());

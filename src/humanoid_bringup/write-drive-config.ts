/**
 * Diagnoses the system polarity (0x200D:00) of devices connected to the currently connected
 * Motion Master session against the values in their matching config files from
 * `drive_configs`, and prints a console table plus a summary.
 *
 * 0x200D:00 is `0` for normal polarity or `1` for inverted polarity.
 *
 * Devices and config files are matched by joint label, not by serial number: every config
 * file and every live device expose a "<number> <joint name>" string at 0x20F2:00
 * (e.g. "104 head_yaw"). By default every config file in `drive_configs` is checked;
 * pass `--device-number` to check only one joint.
 *
 * The console output is split into numbered stages so the process is easy to follow:
 *   Stage 0 (--diagnose and --apply): diagnose current polarity and compare it against the
 *     config files by joint name. Read-only.
 *   Stage 1 (--apply only): ask for confirmation before applying any polarity change.
 *   Stage 2 (--apply only): before cycling the controlword, checks STO (Safe Torque Off — bit 0
 *     of the Safety Statusword's first byte, 0x6621:01) on every device; if STO is active (or
 *     unreadable) on any of them, the run aborts with a warning to release STO, since the
 *     transition cannot succeed while STO is active. Otherwise writes the polarity change
 *     (MISMATCH devices only) and cycles the controlword 0 -> 6 -> 7 on every MATCH/MISMATCH
 *     device. This write is NOT saved/persisted by itself — UNLESS Stage 4 later saves that
 *     same device (see below).
 *   Stage 3 (--apply only): ONLY for devices whose polarity did NOT match their config file
 *     (MISMATCH) that completed Stage 2 — MATCH devices are left alone entirely. Inverting
 *     system polarity at a non-zero angle adds a full revolution to the Actual position value
 *     (e.g. +90° -> -450°, or -90° -> -270°), always correctable by +360° worth of counts on
 *     the home offset. To make that correction exact regardless of prior state, this stage
 *     CHECKS the device's home offset (0x607C:00) and WRITES it to 0 if it isn't already (the
 *     only write this stage makes), then reads the Actual position value (0x6064:00) — for the
 *     printed table only, assuming a 20-bit encoder — and CALCULATES the pending home offset as
 *     +360 degrees worth of counts. Devices needing no correction never occur here since every
 *     MISMATCH/APPLIED device gets the same fixed correction.
 *   Stage 4 (--apply only): ask for confirmation, then — provided restore home position
 *     (0x2005:02) reads 1 on every device that went through Stage 2 (checked and printed as a
 *     precondition; if not, the whole save step is refused) — write the home offset
 *     adjustments Stage 3 calculated and, once verified, SAVE ALL PARAMETERS on that device
 *     (0x1010: Store parameters) — which also persists that device's Stage 2 polarity write,
 *     since save-config persists the entire current parameter set, not just the home offset.
 *     Finally, read and print the Actual position value (0x6064:00), in degrees, for every
 *     device that went through Stage 2 (both MATCH and MISMATCH), regardless of the above.
 *   Stage 5 (--apply only): ask for confirmation, then write the software position limits
 *     (0x607D:01 min, 0x607D:02 max) from each device's matched config file to that device, for
 *     every device that went through Stage 2 whose config file has both values, and — once
 *     verified — SAVE ALL PARAMETERS on that device (0x1010: Store parameters). Prints a
 *     summary in degrees (assuming a 20-bit encoder).
 *   Stage 7 ("Quick and SS1 Configuration", --apply only): applies the same Quick Stop / SS1
 *     safety configuration as `Damping Configuration/configure-damping-parameters.promise.ts`,
 *     to every device that went through Stage 2. Quick stop option code (0x605A:00) and ASC
 *     duration (0x2010:12) are plain CoE objects written via a direct SDO download then SAVED
 *     (0x1010: Store parameters); the three SMM safety parameters (0x2641:01, 0x2650:01,
 *     0x6651:01) are written through the authenticated SMM configuration flow (login ->
 *     transmit -> verify -> validate -> restart) using --username/--password, preserving every
 *     other SMM parameter already on the device. Only devices whose values differ from target
 *     are touched. Read-only sanity checks on the dual-encoder/torque-sensor prerequisites
 *     (0x2112:01, 0x2150:01) are logged but never block the write. Asks for confirmation first.
 *
 * With `--diagnose` only, Stage 0 does NOT write anything to the device — it only reads and
 * reports.
 *
 * Steps (Stage 0):
 *   1. Scan `drive_configs/*.config.csv` and read the 0x20F2:00 label (and 0x200D:00
 *      system polarity) of every file.
 *   2. Discover devices on the network and read the 0x20F2:00 label (and 0x200D:00 system
 *      polarity) of every live device.
 *   3. Pair each config file with the live device sharing the same joint number, and print a
 *      table plus a summary of how many joints matched, mismatched, or couldn't be paired.
 *
 * `--apply` runs Stage 0 first, then Stages 1-5 and 7 (there is no Stage 6). Stage 2's polarity
 * write, Stage 3's home-offset-reset-to-zero write, Stage 4's home offset write, and Stage 5's
 * software position limit write are plain SDO downloads to the device's live parameter set. A
 * device's parameters become persistent if either Stage 4 reaches its save step for that device
 * (requires the device to have gone through Stage 3's correction, the restore-home-position
 * precondition, AND confirmation) or Stage 5 saves it (requires its config file to have both
 * software position limits AND confirmation). Stage 7 saves its own non-safety objects directly
 * and persists its safety objects via the SMM
 * validation flow, independent of Stages 4/5.
 *
 * Usage:
 *   # Check every joint in drive_configs against the live session
 *   npx ts-node src/humanoid_bringup/write-drive-config.ts --diagnose
 *
 *   # Check only one joint
 *   npx ts-node src/humanoid_bringup/write-drive-config.ts --diagnose --device-number 104
 *
 *   # Diagnose, then (after confirmation) write 0x200D:00 on every mismatching device and
 *   # cycle the controlword 0 -> 6 -> 7 on every matching or mismatching device.
 *   # Polarity writes are NOT saved/persisted.
 *   npx ts-node src/humanoid_bringup/write-drive-config.ts --apply
 */

import { readdirSync, readFileSync } from "fs";
import { join } from "path";
import { createInterface } from "readline";

import { Option, program } from "commander";
import {
  ConfigFile,
  ConfigParameter,
  convertGroupedSmmParametersToTransmitData,
  mapByProperty,
  parametersCompareFn,
  resolveAfter,
  resolveSmmOd,
  SmmParameter,
} from "motion-master-client";
import { firstValueFrom, lastValueFrom } from "rxjs";

import { client } from "../init-client";

const DEVICE_LABEL_INDEX = 0x20f2;
const DEVICE_LABEL_SUBINDEX = 0;
const POLARITY_INDEX = 0x200d;
const POLARITY_SUBINDEX = 0;
const CONTROLWORD_INDEX = 0x6040;
const CONTROLWORD_SUBINDEX = 0;
const CONTROLWORD_TRANSITION = [0, 6, 7];
const CONTROLWORD_STEP_DELAY_MS = 200;
const SAFETY_STATUSWORD_INDEX = 0x6621;
const SAFETY_STATUSWORD_BYTE1_SUBINDEX = 1;
const STO_BIT_MASK = 0x01;
const POSITION_ACTUAL_INDEX = 0x6064;
const POSITION_ACTUAL_SUBINDEX = 0;
const HOME_OFFSET_INDEX = 0x607c;
const HOME_OFFSET_SUBINDEX = 0;
const RESTORE_HOME_POSITION_INDEX = 0x2005;
const RESTORE_HOME_POSITION_SUBINDEX = 2;
const ASSUMED_ENCODER_RESOLUTION_BITS = 20;
const COUNTS_PER_REVOLUTION = 2 ** ASSUMED_ENCODER_RESOLUTION_BITS;
const HOME_OFFSET_ADJUSTMENT_DEGREES = 360;
const SOFTWARE_POSITION_LIMIT_INDEX = 0x607d;
const SOFTWARE_POSITION_MIN_SUBINDEX = 1;
const SOFTWARE_POSITION_MAX_SUBINDEX = 2;

// Quick stop / SS1 configuration (Stage 7) — plain CoE objects, written via a direct SDO download.
const QUICK_STOP_NON_SAFETY_PARAMETERS: { index: number; subindex: number; value: number; label: string }[] = [
  { index: 0x605a, subindex: 0, value: 9, label: "Quick stop option code (0x605A:00)" },
  { index: 0x2010, subindex: 0x12, value: 2500, label: "ASC duration (0x2010:12)" },
];

// SMM (safety) object dictionary entries — written via the authenticated SMM configuration flow.
const SS1_SAFETY_PARAMETERS: { index: number; subindex: number; value: number; label: string }[] = [
  { index: 0x2641, subindex: 1, value: 0, label: "Safety parameter (0x2641:01)" },
  { index: 0x2650, subindex: 1, value: 1, label: "Safety parameter (0x2650:01)" },
  { index: 0x6651, subindex: 1, value: 3000, label: "Safety parameter (0x6651:01)" },
];

// Prerequisites the SS1/ASC damping configuration depends on — read-only, never written.
const QUICK_STOP_SANITY_CHECK_PARAMETERS: { index: number; subindex: number; value: number; label: string }[] = [
  { index: 0x2112, subindex: 1, value: 2, label: "Position encoder on Sensor Port 2 (0x2112:01)" },
  { index: 0x2150, subindex: 1, value: 1, label: "Torque sensor configuration (0x2150:01)" },
];

const DEFAULT_CONFIG_DIR = join(__dirname, "drive_configs");

program.addOption(new Option("--diagnose", "Check current polarity for all devices connected to Motion Master against their config files (read-only; does not modify any parameter)").default(false));
program.addOption(new Option("--apply", "Stages 1-7: after diagnosing (Stage 0), ask for confirmation (1), check STO is inactive on all drives then write 0x200D:00 on mismatching devices and cycle the controlword 0 -> 6 -> 7 (2), reset home offset to zero and calculate a +360 degree polarity-inversion correction (3), then ask for confirmation again to check restore home position (0x2005:02 must be 1 on all drives), write those home offsets, and SAVE ALL PARAMETERS on those devices (4, this also persists Stage 2's polarity write), and report actual position for all devices, then confirm and write + save software position limits (0x607D:01/02) from each config file (5), then confirm and apply Quick Stop / SS1 safety configuration via the SMM (7). Implies --diagnose.").default(false));
program.addOption(new Option("--device-number <value>", "Only check the joint matching this number at 0x20F2:00, e.g. 104. If omitted, every config file is checked."));
program.addOption(new Option("--config-dir <path>", "Directory containing the *.config.csv files").default(DEFAULT_CONFIG_DIR));
program.addOption(new Option("-u, --username <value>", "SMM login username, used by Stage 7 (Quick Stop / SS1 configuration)").default("Test"));
program.addOption(new Option("-p, --password <value>", "SMM login password, used by Stage 7 (Quick Stop / SS1 configuration)").default("SomanetSMM"));

program.parse();

const { diagnose, apply, deviceNumber, configDir, username, password, requestTimeout = 10000 } = program.opts();

/** Extracts the leading device number from a "<number> <joint name>" label, e.g. "104 head_yaw" -> "104". */
function extractDeviceNumber(label: string): string | undefined {
  const match = label.trim().match(/^(\d+)\b/);
  return match ? match[1] : undefined;
}

/** Prompts the user on stdin and resolves true only if they type "yes" (case-insensitive). */
function confirm(promptText: string): Promise<boolean> {
  const rl = createInterface({ input: process.stdin, output: process.stdout });
  return new Promise((resolve) => {
    rl.question(promptText, (answer) => {
      rl.close();
      resolve(answer.trim().toLowerCase() === "yes");
    });
  });
}

/** Formats a 0x200D:00 value as "0 (normal)" / "1 (inverted)", passing through anything else (e.g. "-") unchanged. */
function formatPolarity(value: number | string): string {
  if (value === 0) return "0 (normal)";
  if (value === 1) return "1 (inverted)";
  return String(value);
}

/** Prints a "=== Stage N: Title ===" banner to mark the start of a stage. */
function logStage(stage: number, title: string): void {
  console.log(`\n=== Stage ${stage}: ${title} ===\n`);
}

/** Converts a raw encoder count (0x6064:00 / 0x607C:00 units) to degrees, assuming ASSUMED_ENCODER_RESOLUTION_BITS. */
function countsToDegrees(counts: number): number {
  return (counts / COUNTS_PER_REVOLUTION) * 360;
}

/** Converts a degree value to raw encoder counts, assuming ASSUMED_ENCODER_RESOLUTION_BITS. */
function degreesToCounts(degrees: number): number {
  return Math.round((degrees / 360) * COUNTS_PER_REVOLUTION);
}

interface SourceConfig {
  filePath: string;
  label: string;
  filePolarity?: number;
  fileMinPositionLimitCounts?: number;
  fileMaxPositionLimitCounts?: number;
}

/** Scans the config directory and groups every config file by its 0x20F2:00 device number. */
function scanSourceConfigs(dir: string): Map<string, SourceConfig[]> {
  const csvFileNames = readdirSync(dir).filter((name) => name.endsWith(".csv"));

  const byDeviceNumber = new Map<string, SourceConfig[]>();
  for (const fileName of csvFileNames) {
    const filePath = join(dir, fileName);
    const content = readFileSync(filePath, "utf-8");
    const configFile = new ConfigFile(content);

    const labelParam = configFile.parameters.find(
      (p) => p.index === DEVICE_LABEL_INDEX && p.subindex === DEVICE_LABEL_SUBINDEX,
    );
    if (!labelParam) continue;

    const label = String(labelParam.value);
    const number = extractDeviceNumber(label);
    if (!number) continue;

    const polarityParam = configFile.parameters.find(
      (p) => p.index === POLARITY_INDEX && p.subindex === POLARITY_SUBINDEX,
    );
    const minLimitParam = configFile.parameters.find(
      (p) => p.index === SOFTWARE_POSITION_LIMIT_INDEX && p.subindex === SOFTWARE_POSITION_MIN_SUBINDEX,
    );
    const maxLimitParam = configFile.parameters.find(
      (p) => p.index === SOFTWARE_POSITION_LIMIT_INDEX && p.subindex === SOFTWARE_POSITION_MAX_SUBINDEX,
    );

    const entry: SourceConfig = {
      filePath,
      label,
      filePolarity: polarityParam ? Number(polarityParam.value) : undefined,
      fileMinPositionLimitCounts: minLimitParam ? Number(minLimitParam.value) : undefined,
      fileMaxPositionLimitCounts: maxLimitParam ? Number(maxLimitParam.value) : undefined,
    };
    const list = byDeviceNumber.get(number) ?? [];
    list.push(entry);
    byDeviceNumber.set(number, list);
  }
  return byDeviceNumber;
}

interface LiveDevice {
  deviceRef: string;
  label: string;
  deviceAddress: number;
  serialNumber?: string;
}

/** Discovers devices on the network and groups every device exposing 0x20F2:00 by its device number. */
async function scanLiveDevices(): Promise<Map<string, LiveDevice[]>> {
  const devices = await lastValueFrom(client.request.getDevices(requestTimeout));
  console.log(`Discovered ${devices.length} device(s) on the network.`);

  const byDeviceNumber = new Map<string, LiveDevice[]>();
  for (const device of devices) {
    const deviceRef = device.serialNumber ?? device.deviceAddress;
    try {
      const label = await client.request.upload<string>(
        deviceRef,
        DEVICE_LABEL_INDEX,
        DEVICE_LABEL_SUBINDEX,
        false,
        requestTimeout,
      );
      const number = extractDeviceNumber(String(label));
      if (!number) continue;

      const entry: LiveDevice = { deviceRef: String(deviceRef), label: String(label), deviceAddress: device.deviceAddress, serialNumber: device.serialNumber };
      const list = byDeviceNumber.get(number) ?? [];
      list.push(entry);
      byDeviceNumber.set(number, list);
    } catch {
      // Device has no 0x20F2:00 object (e.g. a non-CiA402 device) — skip it.
      continue;
    }
  }
  return byDeviceNumber;
}

type RowStatus = "MATCH" | "MISMATCH" | "NO LIVE DEVICE" | "AMBIGUOUS CONFIG" | "AMBIGUOUS DEVICE" | "NO FILE POLARITY";

interface Row {
  deviceNumber: string;
  label: string;
  filePolarity: number | string;
  devicePolarity: number | string;
  status: RowStatus;
  /** Present only for MATCH/MISMATCH rows, where exactly one live device and file value were resolved. */
  liveDevice?: LiveDevice;
  filePolarityValue?: number;
  fileMinPositionLimitCounts?: number;
  fileMaxPositionLimitCounts?: number;
}

/**
 * Checks whether STO (Safe Torque Off — bit 0 of the Safety Statusword's first byte, 0x6621:01)
 * is active on any device that's about to go through the controlword transition. While STO is
 * active, the controlword transition 0 -> 6 -> 7 cannot succeed (the drive can't leave
 * "Switch on disabled"), so this throws — the caller lets that abort the whole --apply run — if
 * STO is active (or unreadable) on any device, naming which ones.
 */
async function checkStoInactiveOnAllDrives(eligible: Row[]): Promise<void> {
  const blocked: string[] = [];

  for (const row of eligible) {
    const liveDevice = row.liveDevice!;
    try {
      const byte1 = await client.request.upload<number>(
        liveDevice.deviceRef,
        SAFETY_STATUSWORD_INDEX,
        SAFETY_STATUSWORD_BYTE1_SUBINDEX,
        false,
        requestTimeout,
      );
      if ((byte1 & STO_BIT_MASK) !== 0) {
        blocked.push(`${row.deviceNumber} (${row.label})`);
      }
    } catch (err: any) {
      blocked.push(`${row.deviceNumber} (${row.label}) [failed to read STO status: ${err?.message ?? err}]`);
    }
  }

  if (blocked.length > 0) {
    throw new Error(
      `STO (Safe Torque Off) is active on: ${blocked.join(", ")}. ` +
        `The controlword transition 0 -> 6 -> 7 cannot succeed while STO is active. ` +
        `Release STO on the affected device(s) and re-run --apply.`,
    );
  }
}

/**
 * Prompts for confirmation, then processes every row with a resolved polarity comparison
 * (status MATCH or MISMATCH):
 *   - MISMATCH: writes 0x200D:00 to match the config file (a plain SDO download to the
 *     device's live parameter set — NEVER saved/persisted, no 0x1010 store parameters, so it
 *     only lasts until the next power cycle or config reload), verifies it, then cycles the
 *     controlword.
 *   - MATCH: nothing is written; the controlword is still cycled.
 * In both cases the controlword (0x6040:00) is cycled through 0 -> 6 -> 7 (clear -> Shutdown
 * -> Switch On) — but only after confirming STO is inactive on every device (see
 * checkStoInactiveOnAllDrives). Rows with any other status are not eligible and are left
 * untouched.
 */
async function applyPolarity(rows: Row[]): Promise<void> {
  const eligible = rows.filter((r) => r.status === "MATCH" || r.status === "MISMATCH");
  if (eligible.length === 0) {
    console.log("No devices with a resolved polarity comparison — nothing to apply.\n");
    return;
  }

  logStage(1, "Confirm applying polarity changes");

  const mismatchCount = eligible.filter((r) => r.status === "MISMATCH").length;
  console.log(
    `The following ${eligible.length} joint(s) will have their controlword cycled 0 -> 6 -> 7` +
      (mismatchCount > 0 ? `, and ${mismatchCount} of them will first have 0x200D:00 written to match their config file` : "") +
      ":\n",
  );
  console.table(
    eligible.map((r) => ({
      Device: r.deviceNumber,
      Joint: r.label,
      "Current (device)": formatPolarity(r.devicePolarity),
      "New (from file)": formatPolarity(r.filePolarity),
      Action: r.status === "MISMATCH" ? "WRITE + TRANSITION" : "TRANSITION ONLY",
    })),
  );
  console.log(
    "\nMismatching devices get 0x200D:00 written directly to their live parameter set only — NOT saved/persisted.\n" +
      "Every listed device then has its controlword (0x6040:00) cycled 0 -> 6 -> 7.\n",
  );

  const confirmed = await confirm(`Type "yes" to proceed on these ${eligible.length} device(s), anything else to abort: `);
  if (!confirmed) {
    console.log("\nAborted — no changes applied.\n");
    return;
  }

  logStage(2, "Apply polarity changes and controlword transition");

  await checkStoInactiveOnAllDrives(eligible);

  const results: { Device: string; Joint: string; Status: string }[] = [];
  for (const row of eligible) {
    const liveDevice = row.liveDevice!;
    try {
      if (row.status === "MISMATCH") {
        const targetValue = row.filePolarityValue!;
        await client.request.download(liveDevice.deviceRef, POLARITY_INDEX, POLARITY_SUBINDEX, targetValue, undefined, requestTimeout);

        const verified = await client.request.upload<number>(liveDevice.deviceRef, POLARITY_INDEX, POLARITY_SUBINDEX, false, requestTimeout);
        if (verified !== targetValue) {
          results.push({ Device: row.deviceNumber, Joint: row.label, Status: `VERIFY FAILED (read back ${verified})` });
          continue;
        }
      }

      try {
        for (const controlword of CONTROLWORD_TRANSITION) {
          await client.request.download(liveDevice.deviceRef, CONTROLWORD_INDEX, CONTROLWORD_SUBINDEX, controlword, undefined, requestTimeout);
          await resolveAfter(CONTROLWORD_STEP_DELAY_MS);
        }
        results.push({ Device: row.deviceNumber, Joint: row.label, Status: row.status === "MISMATCH" ? "APPLIED" : "TRANSITIONED" });
      } catch (err: any) {
        const prefix = row.status === "MISMATCH" ? "POLARITY WRITTEN, " : "";
        results.push({ Device: row.deviceNumber, Joint: row.label, Status: `${prefix}CONTROLWORD TRANSITION FAILED (${err?.message ?? err})` });
      }
    } catch (err: any) {
      results.push({ Device: row.deviceNumber, Joint: row.label, Status: `WRITE FAILED (${err?.message ?? err})` });
    }
  }

  console.log("=== Apply summary ===");
  console.table(results);
  console.log("Reminder: polarity writes were NOT saved — they apply only until the next power cycle or config reload.\n");

  const stage2StatusByDevice = new Map(results.map((r) => [r.Device, r.Status]));
  const pendingHomeOffsetAdjustments = await resetHomeOffsetAndCalculateInversionCorrection(eligible, stage2StatusByDevice);
  await applyHomeOffsetsAndReportPosition(eligible, pendingHomeOffsetAdjustments);
  await applySoftwarePositionLimits(eligible);
  await applyQuickStopAndSs1Configuration(eligible);
}

interface PendingHomeOffsetAdjustment {
  row: Row;
  positionDegrees: number;
  newHomeOffsetCounts: number;
}

/**
 * Stage 3: ONLY for devices whose polarity did NOT match their config file (status MISMATCH)
 * and that completed Stage 2 (status APPLIED) — MATCH devices are left alone entirely, since
 * their polarity was already correct and nothing was written to them.
 *
 * Inverting system polarity at a non-zero angle adds a full revolution to the Actual position
 * value: e.g. at +90°, inverting polarity moves it to -(360+90) = -450° (not just -90°); at
 * -90°, it moves to -(360-90) = -270° (not +90°). In both cases the fix is the same: add +360°
 * worth of counts to the home offset. For that correction to be exact and independent of
 * whatever was on the device before, this stage first CHECKS the device's current home offset
 * (0x607C:00) and, if it isn't already 0, WRITES it to 0 and verifies the readback — the only
 * write this stage makes. It then reads the Actual position value (0x6064:00) purely for the
 * printed table (the correction itself no longer depends on that value) and CALCULATES the
 * pending home offset as +360 degrees worth of counts (assuming a 20-bit position encoder).
 * Nothing else is written here; Stage 4 applies the calculated home offset. Prints a summary
 * table of the outcome and returns the pending adjustments for Stage 4.
 */
async function resetHomeOffsetAndCalculateInversionCorrection(
  eligible: Row[],
  stage2StatusByDevice: Map<string, string>,
): Promise<PendingHomeOffsetAdjustment[]> {
  logStage(3, "Reset home offset and calculate polarity-inversion correction");

  const mismatched = eligible.filter((r) => r.status === "MISMATCH");
  if (mismatched.length === 0) {
    console.log("No devices had a polarity mismatch — nothing to check for Stage 3.\n");
    return [];
  }

  console.log(
    `[INFO] Inverting system polarity at a non-zero angle adds a full revolution to the Actual\n` +
      `position value, in either direction. Every inverted device's home offset (0x607C:00) is\n` +
      `first reset to 0 (if not already), then corrected by +${HOME_OFFSET_ADJUSTMENT_DEGREES}° worth of counts ` +
      `(assuming a ${ASSUMED_ENCODER_RESOLUTION_BITS}-bit position encoder, ${COUNTS_PER_REVOLUTION} counts/revolution).\n`,
  );

  const pending: PendingHomeOffsetAdjustment[] = [];
  const results: { Device: string; Joint: string; "Home Offset Before (deg)": string; "Position (deg)": string; Status: string }[] = [];

  for (const row of mismatched) {
    const stage2Status = stage2StatusByDevice.get(row.deviceNumber);
    if (stage2Status !== "APPLIED") {
      results.push({ Device: row.deviceNumber, Joint: row.label, "Home Offset Before (deg)": "-", "Position (deg)": "-", Status: "SKIPPED (stage 2 did not complete)" });
      continue;
    }

    const liveDevice = row.liveDevice!;
    try {
      const currentHomeOffsetCounts = await client.request.upload<number>(
        liveDevice.deviceRef,
        HOME_OFFSET_INDEX,
        HOME_OFFSET_SUBINDEX,
        false,
        requestTimeout,
      );

      if (currentHomeOffsetCounts !== 0) {
        await client.request.download(liveDevice.deviceRef, HOME_OFFSET_INDEX, HOME_OFFSET_SUBINDEX, 0, undefined, requestTimeout);
        const verifiedZero = await client.request.upload<number>(liveDevice.deviceRef, HOME_OFFSET_INDEX, HOME_OFFSET_SUBINDEX, false, requestTimeout);
        if (verifiedZero !== 0) {
          results.push({
            Device: row.deviceNumber,
            Joint: row.label,
            "Home Offset Before (deg)": countsToDegrees(currentHomeOffsetCounts).toFixed(2),
            "Position (deg)": "-",
            Status: `RESET TO ZERO FAILED (read back ${verifiedZero})`,
          });
          continue;
        }
      }

      const positionCounts = await client.request.upload<number>(
        liveDevice.deviceRef,
        POSITION_ACTUAL_INDEX,
        POSITION_ACTUAL_SUBINDEX,
        false,
        requestTimeout,
      );
      const positionDegrees = countsToDegrees(positionCounts);
      const newHomeOffsetCounts = degreesToCounts(HOME_OFFSET_ADJUSTMENT_DEGREES);

      pending.push({ row, positionDegrees, newHomeOffsetCounts });
      results.push({
        Device: row.deviceNumber,
        Joint: row.label,
        "Home Offset Before (deg)": countsToDegrees(currentHomeOffsetCounts).toFixed(2),
        "Position (deg)": positionDegrees.toFixed(2),
        Status: `RESET TO 0, +${HOME_OFFSET_ADJUSTMENT_DEGREES}° PENDING IN STAGE 4`,
      });
    } catch (err: any) {
      results.push({ Device: row.deviceNumber, Joint: row.label, "Home Offset Before (deg)": "-", "Position (deg)": "-", Status: `FAILED (${err?.message ?? err})` });
    }
  }

  console.table(results);
  return pending;
}

/**
 * Checks whether the restore home position object (0x2005:02) reads 1 on every device that went
 * through Stage 2 (both MATCH and MISMATCH) — a precondition Stage 4 requires before it will
 * save anything. Prints a table of the result. Returns true only if every device reads back
 * exactly 1; a read failure or any other value counts as a failure.
 */
async function checkRestoreHomePositionOnAllDrives(eligible: Row[]): Promise<boolean> {
  console.log(`Checking restore home position (0x2005:02) on all ${eligible.length} drive(s) before saving...\n`);

  let allOk = true;
  const results: { Device: string; Joint: string; "Restore Home Position (0x2005:02)": number | string; Status: string }[] = [];

  for (const row of eligible) {
    const liveDevice = row.liveDevice!;
    try {
      const value = await client.request.upload<number>(
        liveDevice.deviceRef,
        RESTORE_HOME_POSITION_INDEX,
        RESTORE_HOME_POSITION_SUBINDEX,
        false,
        requestTimeout,
      );
      const ok = value === 1;
      if (!ok) allOk = false;
      results.push({ Device: row.deviceNumber, Joint: row.label, "Restore Home Position (0x2005:02)": value, Status: ok ? "OK" : "NOT 1" });
    } catch (err: any) {
      allOk = false;
      results.push({
        Device: row.deviceNumber,
        Joint: row.label,
        "Restore Home Position (0x2005:02)": "-",
        Status: `READ FAILED (${err?.message ?? err})`,
      });
    }
  }

  console.table(results);
  return allOk;
}

/**
 * Stage 4: asks for confirmation, then, for every device Stage 3 calculated a correction for,
 * checks that restore home position (0x2005:02) reads 1 on every device that went through
 * Stage 2 — if not, the whole save step is refused. Otherwise writes each calculated home
 * offset (0x607C:00, a plain SDO download), verifies it, and — only once verified — SAVES all
 * parameters currently on that device (0x1010: Store parameters) so they persist across a power
 * cycle. This means the polarity change from Stage 2 (previously volatile) becomes persistent
 * too, since save-config persists the device's entire current parameter set, not just the home
 * offset. Finally, regardless of any of the above, reads and prints the Actual position value
 * (0x6064:00), in degrees, for every device that went through Stage 2 (every MATCH and MISMATCH
 * device), so the operator can see where every joint stands after the whole process.
 */
async function applyHomeOffsetsAndReportPosition(eligible: Row[], pending: PendingHomeOffsetAdjustment[]): Promise<void> {
  logStage(4, "Apply home offsets and report actual position");

  if (pending.length === 0) {
    console.log("No home offset corrections to apply.\n");
  } else {
    console.log(`The following ${pending.length} device(s) will have their home offset (0x607C:00) adjusted:\n`);
    console.table(
      pending.map(({ row, positionDegrees, newHomeOffsetCounts }) => ({
        Device: row.deviceNumber,
        Joint: row.label,
        "Position (deg)": positionDegrees.toFixed(2),
        "New Home Offset (counts)": newHomeOffsetCounts,
      })),
    );
    console.log(
      "\nOnce written and verified, ALL parameters currently on each device — including this home\n" +
        "offset and any polarity change from Stage 2 — will be SAVED (0x1010: Store parameters), so\n" +
        "they persist across a power cycle.\n",
    );

    const confirmed = await confirm(
      `Type "yes" to apply these ${pending.length} home offset adjustment(s) and save all parameters, anything else to abort: `,
    );
    if (!confirmed) {
      console.log("\nAborted — no home offset changes applied or saved.\n");
    } else {
      const restoreHomePositionOk = await checkRestoreHomePositionOnAllDrives(eligible);
      if (!restoreHomePositionOk) {
        console.log(
          "\n[ABORT] Restore home position (0x2005:02) is not 1 on all drives — refusing to write or save any home offset.\n",
        );
      } else {
        const results: { Device: string; Joint: string; Status: string }[] = [];
        for (const { row, newHomeOffsetCounts } of pending) {
          const liveDevice = row.liveDevice!;
          try {
            await client.request.download(liveDevice.deviceRef, HOME_OFFSET_INDEX, HOME_OFFSET_SUBINDEX, newHomeOffsetCounts, undefined, requestTimeout);

            const verified = await client.request.upload<number>(liveDevice.deviceRef, HOME_OFFSET_INDEX, HOME_OFFSET_SUBINDEX, false, requestTimeout);
            if (verified !== newHomeOffsetCounts) {
              results.push({ Device: row.deviceNumber, Joint: row.label, Status: `VERIFY FAILED (read back ${verified})` });
              continue;
            }

            try {
              await lastValueFrom(client.request.saveConfig(liveDevice.deviceRef));
              results.push({ Device: row.deviceNumber, Joint: row.label, Status: "APPLIED & SAVED" });
            } catch (err: any) {
              results.push({ Device: row.deviceNumber, Joint: row.label, Status: `APPLIED, SAVE FAILED (${err?.message ?? err})` });
            }
          } catch (err: any) {
            results.push({ Device: row.deviceNumber, Joint: row.label, Status: `WRITE FAILED (${err?.message ?? err})` });
          }
        }

        console.log("=== Home offset apply summary ===");
        console.table(results);
        console.log("Parameters were saved (0x1010) on every successfully applied device — they now persist across a power cycle.\n");
      }
    }
  }

  console.log("Actual position for all devices:\n");
  const positionResults: { Device: string; Joint: string; "Position (deg)": string }[] = [];
  for (const row of eligible) {
    const liveDevice = row.liveDevice!;
    try {
      const positionCounts = await client.request.upload<number>(
        liveDevice.deviceRef,
        POSITION_ACTUAL_INDEX,
        POSITION_ACTUAL_SUBINDEX,
        false,
        requestTimeout,
      );
      positionResults.push({ Device: row.deviceNumber, Joint: row.label, "Position (deg)": countsToDegrees(positionCounts).toFixed(2) });
    } catch (err: any) {
      positionResults.push({ Device: row.deviceNumber, Joint: row.label, "Position (deg)": `FAILED (${err?.message ?? err})` });
    }
  }
  console.table(positionResults);
}

/**
 * Stage 5: applies the software position limits (0x607D:01 min, 0x607D:02 max) from each
 * device's matched config file to that live device, for every device that went through Stage 2
 * (both MATCH and MISMATCH) whose config file has both values. Asks for confirmation before
 * writing anything. This write is NOT saved/persisted by this stage (no 0x1010 store
 * parameters) — it only lasts until the next power cycle or config reload, unless the device
 * was separately saved in Stage 4. Prints a summary table converted to degrees, assuming a
 * 20-bit position encoder.
 */
async function applySoftwarePositionLimits(eligible: Row[]): Promise<void> {
  logStage(5, "Apply software position limits");

  const withLimits = eligible.filter((r) => r.fileMinPositionLimitCounts !== undefined && r.fileMaxPositionLimitCounts !== undefined);
  const withoutLimits = eligible.filter((r) => r.fileMinPositionLimitCounts === undefined || r.fileMaxPositionLimitCounts === undefined);

  if (withLimits.length === 0) {
    console.log("No device's config file has both 0x607D:01 and 0x607D:02 — nothing to apply.\n");
    return;
  }

  console.log(
    `[INFO] Assuming a ${ASSUMED_ENCODER_RESOLUTION_BITS}-bit position encoder ` +
      `(${COUNTS_PER_REVOLUTION} counts/revolution) to convert 0x607D:01/02 to degrees.\n`,
  );

  console.log(`The following ${withLimits.length} device(s) will have their software position limits (0x607D:01/02) written from their config file:\n`);
  console.table(
    withLimits.map((r) => ({
      Device: r.deviceNumber,
      Joint: r.label,
      "Min Limit (deg)": countsToDegrees(r.fileMinPositionLimitCounts!).toFixed(2),
      "Max Limit (deg)": countsToDegrees(r.fileMaxPositionLimitCounts!).toFixed(2),
    })),
  );

  if (withoutLimits.length > 0) {
    console.log(`\n${withoutLimits.length} device(s) have no software position limits in their config file and will be skipped:`);
    for (const r of withoutLimits) console.log(`  - "${r.deviceNumber}" (${r.label})`);
  }
  console.log(
    "\nOnce written and verified, ALL parameters currently on each device will be SAVED\n" +
      "(0x1010: Store parameters), so they persist across a power cycle.\n",
  );

  const confirmed = await confirm(`Type "yes" to write these ${withLimits.length} software position limit(s) and save all parameters, anything else to abort: `);
  if (!confirmed) {
    console.log("\nAborted — no software position limits written or saved.\n");
    return;
  }

  const results: {
    Device: string;
    Joint: string;
    "Min Limit (deg)": string;
    "Max Limit (deg)": string;
    Status: string;
  }[] = [];

  for (const row of withLimits) {
    const liveDevice = row.liveDevice!;
    const minCounts = row.fileMinPositionLimitCounts!;
    const maxCounts = row.fileMaxPositionLimitCounts!;
    try {
      await client.request.download(liveDevice.deviceRef, SOFTWARE_POSITION_LIMIT_INDEX, SOFTWARE_POSITION_MIN_SUBINDEX, minCounts, undefined, requestTimeout);
      await client.request.download(liveDevice.deviceRef, SOFTWARE_POSITION_LIMIT_INDEX, SOFTWARE_POSITION_MAX_SUBINDEX, maxCounts, undefined, requestTimeout);

      const verifiedMin = await client.request.upload<number>(liveDevice.deviceRef, SOFTWARE_POSITION_LIMIT_INDEX, SOFTWARE_POSITION_MIN_SUBINDEX, false, requestTimeout);
      const verifiedMax = await client.request.upload<number>(liveDevice.deviceRef, SOFTWARE_POSITION_LIMIT_INDEX, SOFTWARE_POSITION_MAX_SUBINDEX, false, requestTimeout);

      if (verifiedMin !== minCounts || verifiedMax !== maxCounts) {
        results.push({
          Device: row.deviceNumber,
          Joint: row.label,
          "Min Limit (deg)": countsToDegrees(verifiedMin).toFixed(2),
          "Max Limit (deg)": countsToDegrees(verifiedMax).toFixed(2),
          Status: "VERIFY FAILED",
        });
        continue;
      }

      try {
        await lastValueFrom(client.request.saveConfig(liveDevice.deviceRef));
        results.push({
          Device: row.deviceNumber,
          Joint: row.label,
          "Min Limit (deg)": countsToDegrees(verifiedMin).toFixed(2),
          "Max Limit (deg)": countsToDegrees(verifiedMax).toFixed(2),
          Status: "APPLIED & SAVED",
        });
      } catch (err: any) {
        results.push({
          Device: row.deviceNumber,
          Joint: row.label,
          "Min Limit (deg)": countsToDegrees(verifiedMin).toFixed(2),
          "Max Limit (deg)": countsToDegrees(verifiedMax).toFixed(2),
          Status: `APPLIED, SAVE FAILED (${err?.message ?? err})`,
        });
      }
    } catch (err: any) {
      results.push({
        Device: row.deviceNumber,
        Joint: row.label,
        "Min Limit (deg)": "-",
        "Max Limit (deg)": "-",
        Status: `WRITE FAILED (${err?.message ?? err})`,
      });
    }
  }

  console.log("=== Software position limit summary ===");
  console.table(results);
  console.log("Parameters were saved (0x1010) on every successfully applied device — they now persist across a power cycle.\n");
}

/** Serializes all live SMM parameters to a CSV string, matching the format the SMM configuration flow expects. */
function buildSmmCsvFromParameters(parameters: SmmParameter[]): string {
  return parameters
    .filter((p) => p.index > 0)
    .map((p) => `0x${p.index.toString(16)},${p.subindex},${p.value}`)
    .join("\n");
}

/**
 * Applies a patched SMM configuration CSV to a device via the same flow `configureSmmFromFile`
 * uses (login -> transmit -> load-for-verification -> per-group verify -> load-validation-file ->
 * validate -> refresh-cache), expanded so each OS command can use `commandTimeout` instead of the
 * library's hardcoded 5s default — too short when a device is busy or the network is slow.
 */
async function applySmmSafetyConfig(
  deviceRef: string,
  smmUsername: string,
  smmPassword: string,
  csvContent: string,
  commandTimeout: number,
): Promise<string> {
  const configFile = new ConfigFile(csvContent);

  const parameterStructureVersion = await firstValueFrom(client.request.resolveSmmParameterStructureVersion(deviceRef));
  const smmOd = resolveSmmOd(parameterStructureVersion);

  const groupedValues: Record<string, any[]> = {};
  for (const [group, smmOdParameters] of mapByProperty(smmOd.parameters.safety, "group")) {
    groupedValues[String(group)] = smmOdParameters.map(({ index, subindex }) => {
      const p = configFile.parameters.find((cp) => cp.index === index && cp.subindex === subindex);
      return p?.value ?? 0;
    });
  }
  groupedValues["None"] = [parameterStructureVersion];

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const values: any[] = convertGroupedSmmParametersToTransmitData(groupedValues);

  await lastValueFrom(client.request.logoutFromSmm(deviceRef, commandTimeout));
  await lastValueFrom(client.request.loginToSmmForParameterDownload(deviceRef, smmUsername, smmPassword, commandTimeout));
  await lastValueFrom(client.request.transmitSmmParameters(deviceRef, values, parameterStructureVersion, commandTimeout));

  const verificationStatus = await lastValueFrom(client.request.loadSmmParametersForVerification(deviceRef, undefined, commandTimeout));
  if (verificationStatus.request === "failed") {
    throw new Error(verificationStatus.errorName || verificationStatus.errorDescription || "Verification failed");
  }
  if (verificationStatus.values.join(",") !== values.join(",")) {
    throw new Error("Verification mismatch after loading SMM parameters.");
  }

  const groupEntries = Object.entries(groupedValues).slice(1);
  for (let i = 0; i < groupEntries.length; i++) {
    await lastValueFrom(
      client.request.verifySmmParameters(deviceRef, groupEntries[i][1], i + 1, parameterStructureVersion, commandTimeout),
    );
  }

  const safetyReport = await lastValueFrom(client.request.loadSmmValidationFileAndReadSafetyParametersReportFile(deviceRef, commandTimeout));
  if (!safetyReport) {
    throw new Error("Failed to read the safety parameter report file.");
  }

  await lastValueFrom(
    client.request.validateSmmConfiguration(deviceRef, new TextEncoder().encode(safetyReport), new Date(), smmUsername, smmPassword, commandTimeout),
  );

  // Refresh Motion Master's internal SMM parameter cache.
  await lastValueFrom(client.request.getSmmOdParameters(deviceRef, parameterStructureVersion, "device", false));

  return safetyReport;
}

/**
 * Read-only check of the dual-encoder / torque-sensor prerequisites the SS1/ASC damping
 * configuration depends on. Never writes anything. Returns true only if every check reads at
 * its expected value.
 */
async function runQuickStopSanityChecks(deviceRef: string): Promise<boolean> {
  let allOk = true;
  for (const p of QUICK_STOP_SANITY_CHECK_PARAMETERS) {
    try {
      const current = await client.request.upload<number>(deviceRef, p.index, p.subindex, false, requestTimeout);
      if (current !== p.value) {
        console.log(`    [WARN] ${p.label}: current=${current}, expected=${p.value}. Impedance/ASC configuration may be incomplete.`);
        allOk = false;
      }
    } catch (err: any) {
      console.log(`    [WARN] Failed to read ${p.label}: ${err?.message ?? err}.`);
      allOk = false;
    }
  }
  return allOk;
}

interface QuickStopSs1Preview {
  row: Row;
  quickStopCurrent: number;
  ascDurationCurrent: number;
  smmParameters: SmmParameter[];
  safetyCurrent: (number | undefined)[];
  nonSafetyPending: boolean;
  safetyPending: boolean;
}

/**
 * Stage 7: applies the Quick Stop / SS1 (Safe Stop 1) safety configuration — the same parameters
 * `Damping Configuration/configure-damping-parameters.promise.ts` writes — to every device that
 * went through Stage 2 (both MATCH and MISMATCH):
 *   - Quick stop option code (0x605A:00) and ASC duration (0x2010:12): plain CoE objects,
 *     written via a direct SDO download, then saved (0x1010: Store parameters).
 *   - Three SMM safety parameters (0x2641:01, 0x2650:01, 0x6651:01): written through the
 *     authenticated SMM configuration flow (login -> transmit -> verify -> validate -> restart),
 *     using --username/--password, preserving every other SMM parameter already on the device.
 * Only devices whose current values differ from the target are touched. Read-only sanity checks
 * on the dual-encoder / torque-sensor prerequisites (0x2112:01, 0x2150:01) are logged but never
 * block the write. Asks for confirmation before writing anything.
 */
async function applyQuickStopAndSs1Configuration(eligible: Row[]): Promise<void> {
  logStage(7, "Quick and SS1 Configuration");

  const [quickStopParam, ascDurationParam] = QUICK_STOP_NON_SAFETY_PARAMETERS;
  const previews: QuickStopSs1Preview[] = [];
  const readFailures: { Device: string; Joint: string; Status: string }[] = [];

  for (const row of eligible) {
    const liveDevice = row.liveDevice!;
    try {
      const quickStopCurrent = await client.request.upload<number>(liveDevice.deviceRef, quickStopParam.index, quickStopParam.subindex, false, requestTimeout);
      const ascDurationCurrent = await client.request.upload<number>(liveDevice.deviceRef, ascDurationParam.index, ascDurationParam.subindex, false, requestTimeout);

      if (!liveDevice.serialNumber) {
        readFailures.push({ Device: row.deviceNumber, Joint: row.label, Status: "NO SERIAL NUMBER (cannot address SMM safety parameters)" });
        continue;
      }

      const smmParameters = await lastValueFrom(client.request.getSmmOdParameters(liveDevice.serialNumber, undefined, "device", false));
      const safetyCurrent = SS1_SAFETY_PARAMETERS.map(
        (p) => smmParameters.find((sp) => sp.index === p.index && sp.subindex === p.subindex)?.value as number | undefined,
      );

      const nonSafetyPending = quickStopCurrent !== quickStopParam.value || ascDurationCurrent !== ascDurationParam.value;
      const safetyPending = SS1_SAFETY_PARAMETERS.some((p, i) => safetyCurrent[i] !== p.value);

      previews.push({ row, quickStopCurrent, ascDurationCurrent, smmParameters, safetyCurrent, nonSafetyPending, safetyPending });
    } catch (err: any) {
      readFailures.push({ Device: row.deviceNumber, Joint: row.label, Status: `READ FAILED (${err?.message ?? err})` });
    }
  }

  console.table([
    ...previews.map((p) => ({
      Device: p.row.deviceNumber,
      Joint: p.row.label,
      "Quick Stop (0x605A:00)": `${p.quickStopCurrent} -> ${quickStopParam.value}`,
      "ASC Duration (0x2010:12)": `${p.ascDurationCurrent} -> ${ascDurationParam.value}`,
      "SS1 Safety Params": SS1_SAFETY_PARAMETERS.map((sp, i) => `${p.safetyCurrent[i] ?? "-"}->${sp.value}`).join(", "),
      Status: p.nonSafetyPending || p.safetyPending ? "PENDING" : "ALREADY SET",
    })),
    ...readFailures,
  ]);

  const pending = previews.filter((p) => p.nonSafetyPending || p.safetyPending);
  if (pending.length === 0) {
    console.log("\nAll checked devices already have the target Quick Stop / SS1 configuration — nothing to apply.\n");
    return;
  }

  console.log(
    `\n${pending.length} device(s) need changes. Quick stop / ASC duration are written via a plain SDO\n` +
      `download then saved (0x1010: Store parameters). The three SS1 safety parameters are written via the\n` +
      `authenticated SMM configuration flow (login/transmit/verify/validate/restart) using --username/--password\n` +
      `(currently "${username}"), preserving every other SMM parameter already on the device.\n`,
  );

  const confirmed = await confirm(
    `Type "yes" to apply Quick Stop / SS1 configuration to these ${pending.length} device(s), anything else to abort: `,
  );
  if (!confirmed) {
    console.log("\nAborted — no changes applied.\n");
    return;
  }

  const results: { Device: string; Joint: string; Status: string }[] = [];

  for (const preview of pending) {
    const { row, smmParameters, nonSafetyPending, safetyPending } = preview;
    const liveDevice = row.liveDevice!;
    let sanityNote = "";

    if (nonSafetyPending) {
      try {
        for (const p of QUICK_STOP_NON_SAFETY_PARAMETERS) {
          await client.request.download(liveDevice.deviceRef, p.index, p.subindex, p.value, undefined, requestTimeout);
        }
        console.log(`  [${row.deviceNumber}] Sanity check — dual encoder & torque sensor configuration:`);
        const sanityOk = await runQuickStopSanityChecks(liveDevice.deviceRef);
        if (!sanityOk) sanityNote = " (sanity check warning)";
        await lastValueFrom(client.request.saveConfig(liveDevice.deviceRef));
      } catch (err: any) {
        results.push({ Device: row.deviceNumber, Joint: row.label, Status: `NON-SAFETY WRITE FAILED (${err?.message ?? err})` });
        continue;
      }
    }

    if (safetyPending) {
      try {
        const originalCsv = buildSmmCsvFromParameters(smmParameters);
        const configFile = new ConfigFile(originalCsv);
        for (const p of SS1_SAFETY_PARAMETERS) {
          const existing = configFile.parameters.find((cp) => cp.index === p.index && cp.subindex === p.subindex);
          if (existing) {
            existing.value = p.value;
          } else {
            configFile.parameters.push({ index: p.index, subindex: p.subindex, value: p.value } as ConfigParameter);
          }
        }
        configFile.parameters.sort(parametersCompareFn);
        const patchedCsv = configFile.toString();

        await applySmmSafetyConfig(liveDevice.serialNumber!, username, password, patchedCsv, requestTimeout);
        await lastValueFrom(client.request.triggerSmmRestart(liveDevice.serialNumber!));
        await resolveAfter(2000);
      } catch (err: any) {
        results.push({ Device: row.deviceNumber, Joint: row.label, Status: `SAFETY WRITE FAILED (${err?.message ?? err})${sanityNote}` });
        continue;
      }
    }

    try {
      const verifiedQuickStop = await client.request.upload<number>(liveDevice.deviceRef, quickStopParam.index, quickStopParam.subindex, false, requestTimeout);
      const verifiedAscDuration = await client.request.upload<number>(liveDevice.deviceRef, ascDurationParam.index, ascDurationParam.subindex, false, requestTimeout);
      const verifiedSmmParameters = await lastValueFrom(client.request.getSmmOdParameters(liveDevice.serialNumber!, undefined, "device", false));
      const verifiedSafety = SS1_SAFETY_PARAMETERS.every(
        (p) => verifiedSmmParameters.find((sp) => sp.index === p.index && sp.subindex === p.subindex)?.value === p.value,
      );

      const ok = verifiedQuickStop === quickStopParam.value && verifiedAscDuration === ascDurationParam.value && verifiedSafety;
      results.push({ Device: row.deviceNumber, Joint: row.label, Status: (ok ? "APPLIED & SAVED" : "VERIFY FAILED") + sanityNote });
    } catch (err: any) {
      results.push({ Device: row.deviceNumber, Joint: row.label, Status: `VERIFY FAILED (${err?.message ?? err})` });
    }
  }

  console.log("\n=== Quick Stop / SS1 configuration summary ===");
  console.table(results);
  console.log(
    "Quick stop / ASC duration were saved (0x1010). SS1 safety parameters were validated and persisted via the\n" +
      "SMM configuration flow — both are now persistent on every successfully applied device.\n",
  );
}

async function main(): Promise<void> {
  if (!diagnose && !apply) {
    console.log(
      "\nNothing to do — pass --diagnose to check device polarity against the config files (read-only, does not modify any parameter), " +
        "or --apply to also write mismatching devices after confirmation.\n",
    );
    return;
  }

  logStage(0, `Diagnose system polarity${deviceNumber ? ` for device "${deviceNumber}"` : " for all devices"}`);

  const sourceConfigsByNumber = scanSourceConfigs(configDir);
  const deviceNumbers = [...sourceConfigsByNumber.keys()]
    .filter((n) => !deviceNumber || n === deviceNumber)
    .sort((a, b) => Number(a) - Number(b));

  if (deviceNumbers.length === 0) {
    throw new Error(
      deviceNumber
        ? `No config file in ${configDir} has a 0x20F2:00 label starting with "${deviceNumber}".`
        : `No config file in ${configDir} has a 0x20F2:00 label.`,
    );
  }

  await client.whenReady();
  const liveDevicesByNumber = await scanLiveDevices();

  const rows: Row[] = [];
  for (const number of deviceNumbers) {
    const configs = sourceConfigsByNumber.get(number)!;

    if (configs.length > 1) {
      rows.push({
        deviceNumber: number,
        label: configs.map((c) => c.label).join(", "),
        filePolarity: "-",
        devicePolarity: "-",
        status: "AMBIGUOUS CONFIG",
      });
      continue;
    }

    const config = configs[0];
    const filePolarity = config.filePolarity ?? "-";

    const liveDevices = liveDevicesByNumber.get(number) ?? [];
    if (liveDevices.length === 0) {
      rows.push({ deviceNumber: number, label: config.label, filePolarity, devicePolarity: "-", status: "NO LIVE DEVICE" });
      continue;
    }
    if (liveDevices.length > 1) {
      rows.push({ deviceNumber: number, label: config.label, filePolarity, devicePolarity: "-", status: "AMBIGUOUS DEVICE" });
      continue;
    }

    const liveDevice = liveDevices[0];
    const devicePolarity = await client.request.upload<number>(
      liveDevice.deviceRef,
      POLARITY_INDEX,
      POLARITY_SUBINDEX,
      false,
      requestTimeout,
    );

    let status: RowStatus;
    if (config.filePolarity === undefined) status = "NO FILE POLARITY";
    else if (devicePolarity === config.filePolarity) status = "MATCH";
    else status = "MISMATCH";

    rows.push({
      deviceNumber: number,
      label: config.label,
      filePolarity,
      devicePolarity,
      status,
      liveDevice,
      filePolarityValue: config.filePolarity,
      fileMinPositionLimitCounts: config.fileMinPositionLimitCounts,
      fileMaxPositionLimitCounts: config.fileMaxPositionLimitCounts,
    });
  }

  console.log();
  console.table(
    rows.map((r) => ({
      Device: r.deviceNumber,
      Joint: r.label,
      "File Polarity": formatPolarity(r.filePolarity),
      "Device Polarity": formatPolarity(r.devicePolarity),
      Status: r.status,
    })),
  );

  const counts = rows.reduce<Record<RowStatus, number>>(
    (acc, r) => ({ ...acc, [r.status]: acc[r.status] + 1 }),
    { MATCH: 0, MISMATCH: 0, "NO LIVE DEVICE": 0, "AMBIGUOUS CONFIG": 0, "AMBIGUOUS DEVICE": 0, "NO FILE POLARITY": 0 },
  );

  console.log("=== Summary ===");
  console.log(`  Joints checked:       ${rows.length}`);
  console.log(`  Match:                ${counts.MATCH}`);
  console.log(`  Mismatch:             ${counts.MISMATCH}`);
  console.log(`  No live device:       ${counts["NO LIVE DEVICE"]}`);
  console.log(`  No file polarity:     ${counts["NO FILE POLARITY"]}`);
  console.log(`  Ambiguous config:     ${counts["AMBIGUOUS CONFIG"]}`);
  console.log(`  Ambiguous device:     ${counts["AMBIGUOUS DEVICE"]}`);
  console.log("================\n");

  const unmatchedLiveDevices = [...liveDevicesByNumber.entries()].filter(([number]) => !sourceConfigsByNumber.has(number));
  if (unmatchedLiveDevices.length > 0) {
    console.log("Live devices with no matching config file in drive_configs:");
    for (const [number, devices] of unmatchedLiveDevices) {
      for (const d of devices) {
        console.log(`  - "${number}" addr=0x${d.deviceAddress.toString(16).toUpperCase()} (label: "${d.label}")`);
      }
    }
    console.log();
  }

  if (apply) {
    await applyPolarity(rows);
  }
}

main()
  .catch((err: any) => {
    console.error(`[FATAL] ${err?.message ?? err}`);
    process.exit(1);
  })
  .finally(() => client.closeSockets());

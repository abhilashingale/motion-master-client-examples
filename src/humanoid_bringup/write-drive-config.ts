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
 *   Stage 1 (--apply with --assign-names): write each drive's joint label (0x20F2:00) from the
 *     `position_<NN>_*.csv` files, by position in the EtherCAT chain, so a robot whose drives
 *     were never labelled can be named in the same run. The files come from the IgH master
 *     (positions count from 0 and include slaves that are not drives, e.g. EtherCAT switches),
 *     while Motion Master here runs SOEM and only reports the drives: pass the IgH positions of
 *     the slaves that are not drives with --ignore_ecat_indices (Roger-3: 0,3,18). A file's
 *     position becomes a drive index by taking out the ignored positions before it. Prints a
 *     large warning that the EtherCAT network must be wired exactly the same as on the robot the
 *     files came from and that cabling is assumed monotonic. After the labels are saved, Stage 0
 *     is run again so the later stages see the new names.
 *   Stage 2 (--apply only, unless --skip-load-configs): load each paired config file onto its drive,
 *     so the robot gets its configuration in the same run (this replaces `load-drive-configs.ts`).
 *     The Home Offset (0x607C) is NEVER loaded — the script prints that it is leaving it out, and
 *     the file's value for it — because no home offset is used: Stage 4 homes every joint where it
 *     is and the Home Offset must stay 0. --exclude adds more objects to leave out (e.g. the
 *     per-drive commutation offset 0x2001:00). The strategy is "merge": Motion Master reads the
 *     drive's existing config.csv, applies the file's rows on top and writes the result back, so an
 *     excluded object keeps the value the drive already has. SMM safety parameters and the joint
 *     label 0x20F2:00 are always left out. Asks for confirmation, and repeats Stage 0 afterwards.
 *   Stage 3 (--apply only): ask for confirmation, then check STO (Safe Torque Off — bit 0 of the
 *     Safety Statusword's first byte, 0x6621:01) on every device; if STO is active (or
 *     unreadable) on any of them, the run aborts with a warning to release STO, since the
 *     transition cannot succeed while STO is active. Otherwise writes the system polarity from
 *     the config file (MISMATCH devices only) and cycles the controlword 0 -> 6 -> 7 on every
 *     MATCH/MISMATCH device. The polarity is saved in Stage 4.
 *   Stage 4 (--apply only): for every joint that completed Stage 3, centre the Encoder 2 single
 *     turn range and home the joint there. Read --samples (5-10) samples of Encoder 2 feedback
 *     Adjusted position (0x2113:02) per drive (refusing a moving drive) and print a table. After
 *     confirmation, joint by joint (its assigned name is printed, followed by a pause): make sure
 *     the Home Offset (0x607C:00) is 0, set 0x2112:06 = 0, read 0x2113:02, calculate the offset
 *     (IF adjusted < 2^19 THEN adjusted + 2^19 ELSE adjusted - 2^19; 0x2113:01 is never used),
 *     write it with Restore home position (0x2005:02) = 1, save all parameters, read 0x2113:02 back (it must
 *     be 2^19, otherwise the previous values are restored), then home at the current position with
 *     method 37 (Homing method 0x6098:00 = 37, controlword 6 -> 7 -> 15 -> 31). Homing is not
 *     meant to move the joint, but the drive IS energized. NO home offset correction is made: when
 *     the homing is done the Home Offset must still be 0 and the Actual position close to 0,
 *     otherwise the run FAILS LOUDLY and stops. Each drive is returned to Switched on disabled.
 *     Start with the robot mechanically zeroed (or at the zero you want).
 *   Stage 5 (--apply only): ask for confirmation, then SAVE ALL PARAMETERS on every homed device
 *     (0x1010: Store parameters) so the homed position persists, and print the Actual position
 *     (0x6064:00) and Home Offset (0x607C:00); a non-zero Home Offset fails loudly.
 *   Stage 6 (--apply only): ask for confirmation, then write the software position limits
 *     (0x607D:01 min, 0x607D:02 max) from each device's matched config file to that device, for
 *     every device that went through Stage 3 whose config file has both values, and — once
 *     verified — SAVE ALL PARAMETERS on that device (0x1010: Store parameters). Prints a
 *     summary in degrees (assuming a 20-bit encoder).
 *   Stage 7 ("Quick and SS1 Configuration", --apply only): applies the same Quick Stop / SS1
 *     safety configuration as `Damping Configuration/configure-damping-parameters.promise.ts`,
 *     to every device that went through Stage 3. Quick stop option code (0x605A:00) and ASC
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
 * `--apply` runs Stage 0 first, then Stages 1-7. A failure in
 * Stages 1-5 throws and stops the run, so the later stages never run on a half-commissioned
 * robot. Stage 4 and 5 make the parameter writes persistent (0x1010); the polarity is written in
 * Stage 3 and saved in Stage 4.
 *
 * This script ENERGIZES the drives (Stage 3 cycles the controlword to Switch on; Stage 4 enables
 * operation for homing). Run it yourself at the robot with the approved E-stop/STO within reach.
 *
 * Usage:
 *   # Check every joint in drive_configs against the live session
 *   npx ts-node src/humanoid_bringup/write-drive-config.ts --diagnose
 *
 *   # Check only one joint
 *   npx ts-node src/humanoid_bringup/write-drive-config.ts --diagnose --device-number 104
 *
 *   # Full commissioning of a new robot, starting with the names (Roger-3: IgH 0, 3, 18 are switches)
 *   npx ts-node src/humanoid_bringup/write-drive-config.ts --apply --assign-names --ignore_ecat_indices 0,3,18
 *
 *   # Drives already named
 *   npx ts-node src/humanoid_bringup/write-drive-config.ts --apply
 */

import { readdirSync, readFileSync } from "fs";
import { join } from "path";
import { createInterface } from "readline";

import { Option, program } from "commander";
import {
  ConfigFile,
  ConfigParameter,
  isSmmParameterIndex,
  LoadConfigStrategy,
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
const STATUSWORD_INDEX = 0x6041;
const MODES_OF_OPERATION_INDEX = 0x6060;
const HOMING_METHOD_INDEX = 0x6098;
const ENCODER2_ST_OFFSET_INDEX = 0x2112;
const ENCODER2_ST_OFFSET_SUBINDEX = 6;
const ENCODER2_FEEDBACK_INDEX = 0x2113;
const ENCODER2_ADJUSTED_SUBINDEX = 2;
const ENCODER_SAMPLE_INTERVAL_MS = 100;
const HOMING_MODE = 6;
const HOMING_METHOD_CURRENT_POSITION = 37;
const HOMING_TIMEOUT_MS = 10000;
const SW_FAULT = 1 << 3;
const SW_OPERATION_ENABLED = 1 << 2;
const SW_HOMING_ATTAINED = 1 << 12;
const SW_HOMING_ERROR = 1 << 13;
const CW_SHUTDOWN = 0x06;
const CW_SWITCH_ON = 0x07;
const CW_ENABLE_OPERATION = 0x0f;
const CW_START_HOMING = 0x1f;
const CW_DISABLE_VOLTAGE = 0x00;
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

/** `0x2001:00` -> that subindex only; `0x2001` -> every subindex of the object. */
interface ExclusionRule {
  index: number;
  subindex?: number;
  raw: string;
  reason: string;
}

function parseExclusions(raw: string, reason: string): ExclusionRule[] {
  return raw
    .split(",")
    .map((t) => t.trim())
    .filter((t) => t.length > 0)
    .map((token) => {
      const [indexPart, subindexPart] = token.split(":");
      const index = parseInt(indexPart, 16);
      if (isNaN(index)) throw new Error(`Invalid ${reason} entry "${token}". Expected e.g. 0x2001:00 or 0x2001.`);
      if (subindexPart === undefined) return { index, raw: token, reason };
      const subindex = parseInt(subindexPart, 10);
      if (isNaN(subindex)) throw new Error(`Invalid subindex in ${reason} entry "${token}".`);
      return { index, subindex, raw: token, reason };
    });
}

/**
 * Never loaded by Stage 2: the Home Offset (0x607C). Stage 4 homes every joint at its current
 * position and NO home offset is used — it must be 0 — so a value carried by a config file (some
 * exported ones hold 1048576, i.e. 360 degrees) must not reach the drive.
 */
const ALWAYS_EXCLUDED_FROM_LOAD: ExclusionRule[] = [
  { index: 0x607c, raw: "0x607C", reason: "Home Offset must stay 0 (no home offset is used)" },
];

program.addOption(new Option("--diagnose", "Check current polarity for all devices connected to Motion Master against their config files (read-only; does not modify any parameter)").default(false));
program.addOption(new Option("--apply", "Stages 1-7: after diagnosing (Stage 0), optionally assign names by slave position (1, with --assign-names), load the config files without 0x607C (2), confirm then write 0x200D:00 from the config files on mismatching devices and cycle the controlword 0 -> 6 -> 7 (2), then set the Encoder 2 single turn offset so the encoder is in the middle of its single-turn range and home each joint there with method 37, one by one, failing loudly if the Home Offset is not 0 (3), save and report actual position (4), write + save software position limits (0x607D:01/02) from each config file (5), then apply Quick Stop / SS1 safety configuration via the SMM (7). Implies --diagnose.").default(false));
program.addOption(new Option("--device-number <value>", "Only check the joint matching this number at 0x20F2:00, e.g. 104. If omitted, every config file is checked."));
program.addOption(new Option("--assign-names", "Stage 1: write each drive's joint label (0x20F2:00) from the position_<NN>_*.csv files, by position in the EtherCAT chain (use --ignore_ecat_indices)").default(false));
program.addOption(
  new Option("--ignore_ecat_indices <list>", "Stage 1: comma-separated IgH positions of slaves that are not drives (e.g. EtherCAT switches) and have no config file, e.g. 0,3,18")
    .default([] as number[], "none")
    .argParser((raw: string) =>
      raw.split(",").map((t) => t.trim()).filter((t) => t.length > 0).map((t) => {
        const n = parseInt(t, 10);
        if (!Number.isInteger(n) || n < 0) throw new Error(`Invalid --ignore_ecat_indices entry "${t}".`);
        return n;
      }),
    ),
);
program.addOption(new Option("--skip-load-configs", "Stage 2: do not load the config files onto the drives").default(false));
program.addOption(
  new Option(
    "--exclude <ids>",
    "Stage 2: comma-separated objects to leave out of every config load IN ADDITION to 0x607C (Home Offset), e.g. 0x2001:00,0x2112 (an entry without a subindex excludes every subindex)",
  ).argParser((raw: string) => parseExclusions(raw, "--exclude")),
);
program.addOption(new Option("--samples <n>", "Stage 4: samples of the Encoder 2 position to read per drive (5-10)").default(10).argParser((v: string) => {
  const n = parseInt(v, 10);
  if (!Number.isInteger(n) || n < 5 || n > 10) throw new Error(`Invalid --samples "${v}". Expected 5-10.`);
  return n;
}));
program.addOption(new Option("--resolution-bits <n>", "Stage 4: Encoder 2 single-turn resolution in bits (the middle is 2^(n-1))").default(20).argParser((v: string) => {
  const n = parseInt(v, 10);
  if (!Number.isInteger(n) || n < 2 || n > 31) throw new Error(`Invalid --resolution-bits "${v}".`);
  return n;
}));
program.addOption(new Option("--max-spread <counts>", "Stage 4: refuse a drive whose raw Encoder 2 position varies by more than this over the samples").default(16).argParser(Number));
program.addOption(new Option("--tolerance <counts>", "Stage 4: how far from the middle the read-back adjusted position may be").default(4).argParser(Number));
program.addOption(new Option("--home-tolerance-deg <deg>", "Stage 4: largest |Actual position| after homing that counts as close to 0").default(0.5).argParser(Number));
program.addOption(new Option("--pause-ms <ms>", "Stage 4: pause after announcing each joint, before it is processed").default(2000).argParser((v: string) => {
  const n = parseInt(v, 10);
  if (!Number.isInteger(n) || n < 0) throw new Error(`Invalid --pause-ms "${v}".`);
  return n;
}));
program.addOption(new Option("--config-dir <path>", "Directory containing the *.config.csv files").default(DEFAULT_CONFIG_DIR));
program.addOption(new Option("-u, --username <value>", "SMM login username, used by Stage 7 (Quick Stop / SS1 configuration)").default("Test"));
program.addOption(new Option("-p, --password <value>", "SMM login password, used by Stage 7 (Quick Stop / SS1 configuration)").default("SomanetSMM"));

program.parse();

const {
  diagnose,
  apply,
  deviceNumber,
  configDir,
  username,
  password,
  assignNames: assignNamesRequested,
  skipLoadConfigs,
  exclude: extraLoadExclusions = [] as ExclusionRule[],
  ignore_ecat_indices: ignoredEcatIndices,
  samples: encoderSamples,
  resolutionBits,
  maxSpread,
  tolerance: centreTolerance,
  homeToleranceDeg,
  pauseMs,
  requestTimeout = 10000,
} = program.opts() as {
  diagnose: boolean;
  apply: boolean;
  deviceNumber?: string;
  configDir: string;
  username: string;
  password: string;
  assignNames: boolean;
  skipLoadConfigs: boolean;
  exclude?: ExclusionRule[];
  ignore_ecat_indices: number[];
  samples: number;
  resolutionBits: number;
  maxSpread: number;
  tolerance: number;
  homeToleranceDeg: number;
  pauseMs: number;
  requestTimeout?: number;
};

const ENCODER2_RESOLUTION = 2 ** resolutionBits;
const ENCODER2_HALF = ENCODER2_RESOLUTION / 2;

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
  /** The config file this row was paired with (rows with a live device only). */
  filePath?: string;
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

/** Reads a numeric object from a live device. */
const readObject = (d: LiveDevice, index: number, subindex: number) =>
  client.request.upload<number>(d.deviceRef, index, subindex, false, requestTimeout);

/** Writes a numeric object to a live device (a plain SDO download). */
const writeObject = (d: LiveDevice, index: number, subindex: number, value: number) =>
  client.request.download(d.deviceRef, index, subindex, value, undefined, requestTimeout);

/** Writes a numeric object and reads it back; throws when the drive does not hold the written value. */
async function writeAndVerify(d: LiveDevice, index: number, subindex: number, value: number): Promise<void> {
  await writeObject(d, index, subindex, value);
  const readBack = await readObject(d, index, subindex);
  if (readBack !== value) {
    throw new Error(`0x${index.toString(16).toUpperCase()}:${String(subindex).padStart(2, "0")} wrote ${value} but reads back ${readBack}`);
  }
}

/** Prints a block of `!` around the given lines, for warnings that must not be missed. */
function banner(lines: string[]): void {
  const inner = Math.max(...lines.map((l) => l.length)) + 4;
  const bar = "!".repeat(inner + 4);
  const blank = `!!${" ".repeat(inner)}!!`;
  const body = lines.map((l) => `!!  ${l.padEnd(inner - 2)}!!`);
  console.warn(`\n${[bar, bar, blank, ...body, blank, bar, bar].join("\n")}\n`);
}

// ---------------------------------------------------------------------------------------------
// Stage 1: assign names by slave position
// ---------------------------------------------------------------------------------------------

interface NameSource {
  /** The IgH position from the filename (`position_<NN>_...`). */
  filePosition: number;
  fileName: string;
  /** The "<number> <joint name>" string from the file's 0x20F2:00 row. */
  label: string;
}

/** Reads every `position_NN_*.csv` in `dir` and returns the label each one carries, by file position. */
function scanNameSources(dir: string): { sources: Map<number, NameSource>; problems: string[] } {
  const sources = new Map<number, NameSource>();
  const problems: string[] = [];

  for (const fileName of readdirSync(dir).filter((n) => n.endsWith(".csv")).sort()) {
    const match = fileName.match(/^position_(\d+)[_-]/i);
    if (!match) {
      problems.push(`${fileName}: filename does not start with "position_<NN>_", ignored`);
      continue;
    }
    const position = parseInt(match[1], 10);

    const configFile = new ConfigFile(readFileSync(join(dir, fileName), "utf-8"));
    const labelParam = configFile.parameters.find(
      (p) => p.index === DEVICE_LABEL_INDEX && p.subindex === DEVICE_LABEL_SUBINDEX,
    );
    const label = labelParam ? String(labelParam.value).trim() : "";
    if (!extractDeviceNumber(label)) {
      problems.push(`${fileName}: no usable 0x20F2:00 label ("<number> <joint name>") in the file, ignored`);
      continue;
    }

    const existing = sources.get(position);
    if (existing) {
      problems.push(`file position ${position}: claimed by both ${existing.fileName} and ${fileName}`);
      continue;
    }
    sources.set(position, { filePosition: position, fileName, label });
  }

  return { sources, problems };
}

/** The IgH position of a file -> the 0-based index of its drive in bus order, with the ignored positions taken out. */
function driveIndexOf(igHPosition: number): number {
  return igHPosition - ignoredEcatIndices.filter((s) => s < igHPosition).length;
}

/**
 * Stage 1 (--apply with --assign-names): writes each drive's joint label (0x20F2:00) from the
 * config files, by position in the EtherCAT chain, so a robot whose drives were never labelled
 * can be named in the same run. Returns true when at least one label was written, so the caller
 * re-runs the Stage 0 pairing.
 *
 * The files are generated with the IgH master, which counts from 0 and counts every slave on the
 * bus, including those that are not drives (e.g. Beckhoff EtherCAT switches). Motion Master here
 * runs SOEM, which only reports the drives. --ignore_ecat_indices lists the IgH positions of the
 * slaves that are not drives; the file position is converted to a drive index by taking out the
 * ignored positions that come before it (with 0,3,18: position_01 -> 1st drive, position_04 -> 3rd,
 * position_19 -> 17th). Devices are sorted by position, so SOEM's 1-based and IgH's 0-based
 * numbering make no difference. Only correct when the cabling is monotonic.
 */
async function assignNames(): Promise<boolean> {
  logStage(1, "Assign names by slave position");

  if (!assignNamesRequested) {
    console.log("Skipped — pass --assign-names (with --ignore_ecat_indices) to write the joint labels from the config files.\n");
    return false;
  }

  banner([
    "Applying this step assumes EtherCAT network is wired exactly same between the robots",
    ignoredEcatIndices.length > 0
      ? `Assuming monotonic cabling and position ${ignoredEcatIndices.join(", ")} as EtherCAT switch positions`
      : "Assuming monotonic cabling and no ignored EtherCAT positions (see --ignore_ecat_indices)",
  ]);

  const { sources, problems } = scanNameSources(configDir);
  console.log(`Found ${sources.size} config file(s) with a position and a label.`);
  for (const p of problems) console.warn(`  [WARN] ${p}`);
  if (sources.size === 0) throw new Error(`No usable position_<NN>_*.csv files in ${configDir}.`);

  const devices = await lastValueFrom(client.request.getDevices(requestTimeout));
  const byBusOrder = [...devices].sort((a, b) => (a.position ?? 0) - (b.position ?? 0));
  console.log(`Discovered ${byBusOrder.length} drive(s) on the network.\n`);
  if (byBusOrder.length !== sources.size) {
    console.warn(
      `[WARN] The network has ${byBusOrder.length} drive(s) but there are ${sources.size} config file(s). ` +
        `The robot is probably not the one the files came from.\n`,
    );
  }

  interface NameRow {
    filePosition: number;
    driveNumber?: number;
    deviceRef?: string | number;
    deviceAddress?: number;
    currentLabel: string;
    newLabel: string;
    fileName: string;
    skip?: string;
  }

  const rows: NameRow[] = [];
  for (const source of [...sources.values()].sort((a, b) => a.filePosition - b.filePosition)) {
    if (deviceNumber && extractDeviceNumber(source.label) !== deviceNumber) continue;

    if (ignoredEcatIndices.includes(source.filePosition)) {
      rows.push({ filePosition: source.filePosition, currentLabel: "-", newLabel: source.label, fileName: source.fileName, skip: "FILE AT AN IGNORED POSITION" });
      continue;
    }

    const driveIndex = driveIndexOf(source.filePosition);
    const device = byBusOrder[driveIndex];
    if (!device) {
      rows.push({ filePosition: source.filePosition, driveNumber: driveIndex + 1, currentLabel: "-", newLabel: source.label, fileName: source.fileName, skip: "NO DRIVE AT THIS POSITION" });
      continue;
    }

    const deviceRef = device.serialNumber ?? device.deviceAddress;
    let currentLabel = "(unreadable)";
    try {
      currentLabel = String(
        await client.request.upload<string>(deviceRef, DEVICE_LABEL_INDEX, DEVICE_LABEL_SUBINDEX, false, requestTimeout),
      );
    } catch {
      // A drive that was never labelled — the write below decides.
    }
    rows.push({
      filePosition: source.filePosition,
      driveNumber: driveIndex + 1,
      deviceRef,
      deviceAddress: device.deviceAddress,
      currentLabel,
      newLabel: source.label,
      fileName: source.fileName,
    });
  }
  if (rows.length === 0) throw new Error(`No config file matches --device-number ${deviceNumber}.`);

  // Two files with the same joint number would make every later step ambiguous.
  const seen = new Map<string, NameRow>();
  for (const row of rows) {
    const key = extractDeviceNumber(row.newLabel) ?? row.newLabel;
    const other = seen.get(key);
    if (other) row.skip = row.skip ?? `DUPLICATE JOINT NUMBER (also file position ${other.filePosition})`;
    else seen.set(key, row);
  }

  console.table(
    rows.map((r) => ({
      "File position (IgH)": r.filePosition,
      "Drive # (SOEM)": r.driveNumber ?? "-",
      "Device address": r.deviceAddress !== undefined ? `0x${r.deviceAddress.toString(16).toUpperCase()}` : "-",
      "Current label": r.currentLabel,
      "New label": r.newLabel,
      File: r.fileName,
      Status: r.skip ?? (r.currentLabel === r.newLabel ? "ALREADY SET" : "PENDING"),
    })),
  );

  const mapped = new Set(rows.map((r) => r.driveNumber).filter((n): n is number => n !== undefined));
  const untouched = byBusOrder.map((_, i) => i + 1).filter((n) => !mapped.has(n));
  if (!deviceNumber && untouched.length > 0) {
    console.log(`Drive(s) with no config file, left untouched: ${untouched.join(", ")}\n`);
  }

  const problemsInTable = rows.filter((r) => r.skip);
  const toWrite = rows.filter((r) => !r.skip && r.currentLabel !== r.newLabel);
  if (toWrite.length === 0) {
    console.log("No label needs to be written.\n");
    if (problemsInTable.length > 0) throw new Error(`${problemsInTable.length} config file(s) could not be paired with a drive (see the table).`);
    return false;
  }

  banner([
    "Applying this step assumes EtherCAT network is wired exactly same between the robots",
    ignoredEcatIndices.length > 0
      ? `Assuming monotonic cabling and position ${ignoredEcatIndices.join(", ")} as EtherCAT switch positions`
      : "Assuming monotonic cabling and no ignored EtherCAT positions (see --ignore_ecat_indices)",
  ]);
  console.log(
    `${toWrite.length} label(s) will be written to 0x20F2:00 and saved (0x1010: Store parameters) on the drive\n` +
      `at each position above. Check the table: a drive at the wrong position gets the wrong name.\n`,
  );
  if (!(await confirm(`Type "yes" to assign these ${toWrite.length} name(s), anything else to abort: `))) {
    throw new Error("Aborted at Stage 1 — no names assigned, and the later stages need them.");
  }

  const results: { "File position (IgH)": number; "New label": string; Status: string }[] = [];
  for (const [i, row] of toWrite.entries()) {
    console.log(`[${i + 1}/${toWrite.length}] Assigning "${row.newLabel}" to drive #${row.driveNumber} (file position ${row.filePosition})...`);
    const result = (Status: string) => {
      console.log(`[${i + 1}/${toWrite.length}] ${row.newLabel}: ${Status}`);
      results.push({ "File position (IgH)": row.filePosition, "New label": row.newLabel, Status });
    };
    try {
      await client.request.download(row.deviceRef!, DEVICE_LABEL_INDEX, DEVICE_LABEL_SUBINDEX, row.newLabel, "stringValue", requestTimeout);
      const readBack = String(
        await client.request.upload<string>(row.deviceRef!, DEVICE_LABEL_INDEX, DEVICE_LABEL_SUBINDEX, false, requestTimeout),
      );
      if (readBack !== row.newLabel) {
        result(`VERIFY FAILED (read back "${readBack}")`);
        continue;
      }
      try {
        await lastValueFrom(client.request.saveConfig(row.deviceRef!));
        result("ASSIGNED & SAVED");
      } catch (err: any) {
        result(`ASSIGNED, SAVE FAILED (${err?.message ?? err})`);
      }
    } catch (err: any) {
      result(`WRITE FAILED (${err?.message ?? err})`);
    }
  }

  console.log("\n=== Assign names summary ===");
  console.table(results);
  const failed = results.filter((r) => r.Status !== "ASSIGNED & SAVED");
  if (failed.length > 0) throw new Error(`${failed.length} name(s) were not assigned and saved — see the table.`);
  return true;
}

// ---------------------------------------------------------------------------------------------
// Stage 2: load the config files, without 0x607C
// ---------------------------------------------------------------------------------------------

const objectId = (p: Pick<ConfigParameter, "index" | "subindex">) =>
  `0x${p.index.toString(16).toUpperCase().padStart(4, "0")}:${String(p.subindex).padStart(2, "0")}`;

/** Merge keeps what the drive already has for every object the file does not carry (or that is excluded). */
const LOAD_CONFIG_STRATEGY: LoadConfigStrategy = "merge";

/**
 * Stage 2: loads each paired config file onto its drive, leaving out the Home Offset (0x607C)
 * and anything given with --exclude, and says so on the console.
 *
 * The strategy is "merge": Motion Master reads the drive's existing config.csv, applies the
 * file's rows on top and writes the result back, so an excluded object keeps the value the drive
 * already has. Always dropped as well: the SMM safety parameters (the safety layer ignores plain
 * writes to them) and the joint label 0x20F2:00, so a label can never end up on the wrong drive.
 *
 * Returns true when at least one config was loaded, so the caller re-runs the Stage 0 pairing.
 */
async function loadConfigs(rows: Row[]): Promise<boolean> {
  logStage(2, "Load the config files onto the drives, without 0x607C (Home Offset)");

  if (skipLoadConfigs) {
    console.log("Skipped (--skip-load-configs).\n");
    return false;
  }

  const exclusions = [...ALWAYS_EXCLUDED_FROM_LOAD, ...extraLoadExclusions];
  console.warn("[EXCLUDE] These objects are NOT loaded on any drive:");
  for (const rule of exclusions) console.warn(`  - ${rule.raw}${rule.subindex === undefined ? " (every subindex)" : ""}: ${rule.reason}`);
  console.warn("  - SMM safety parameters and the joint label 0x20F2:00: always left out\n");

  interface LoadTarget {
    row: Row;
    configFile: ConfigFile;
    total: number;
    dropped: { p: ConfigParameter; reason: string }[];
  }
  const targets: LoadTarget[] = [];
  for (const row of rows) {
    if (!row.liveDevice || !row.filePath) continue;
    const configFile = new ConfigFile(readFileSync(row.filePath, "utf-8"));
    const total = configFile.numberOfParameters;
    const dropped: LoadTarget["dropped"] = [];
    configFile.parameters = configFile.parameters.filter((p) => {
      const rule = exclusions.find((r) => r.index === p.index && (r.subindex === undefined || r.subindex === p.subindex));
      if (rule) {
        dropped.push({ p, reason: `excluded ${rule.raw}` });
        return false;
      }
      if (isSmmParameterIndex(p.index)) {
        dropped.push({ p, reason: "SMM safety parameter" });
        return false;
      }
      if (p.index === DEVICE_LABEL_INDEX && p.subindex === DEVICE_LABEL_SUBINDEX) {
        dropped.push({ p, reason: "joint label" });
        return false;
      }
      return true;
    });
    targets.push({ row, configFile, total, dropped });
  }

  if (targets.length === 0) {
    console.log("No device is paired with a config file — nothing to load.\n");
    return false;
  }

  console.table(
    targets.map((t) => ({
      Device: t.row.deviceNumber,
      Joint: t.row.label,
      File: t.row.filePath!.split("/").pop(),
      "To send": `${t.configFile.parameters.length} of ${t.total}`,
      "Excluded": t.dropped.length,
    })),
  );
  for (const t of targets) {
    for (const { p, reason } of t.dropped) {
      if (reason.startsWith("excluded")) console.log(`  [EXCLUDE] ${t.row.label}: ${objectId(p)} = ${p.value} not loaded (${reason})`);
    }
  }

  console.log(`\nStrategy: ${LOAD_CONFIG_STRATEGY} — what the drive already has is kept for every object that is not sent.\n`);
  if (!(await confirm(`Type "yes" to load these ${targets.length} config(s), anything else to skip loading: `))) {
    console.log("\nSkipped — no config loaded.\n");
    return false;
  }

  const results: { Device: string; Joint: string; Status: string }[] = [];
  console.log();
  for (const [i, t] of targets.entries()) {
    const progress = `[${i + 1}/${targets.length}]`;
    const fileName = t.row.filePath!.split("/").pop();
    console.log(
      `${progress} Loading config onto ${t.row.label}: ${fileName} ` +
        `(${t.configFile.parameters.length} parameter(s), ${t.dropped.length} excluded)...`,
    );
    const startedAt = Date.now();
    try {
      await lastValueFrom(
        client.request.loadConfig(t.row.liveDevice!.deviceRef, t.configFile.toString(), LOAD_CONFIG_STRATEGY, { count: 10, delay: 500 }),
      );
      console.log(`${progress} ${t.row.label}: loaded in ${((Date.now() - startedAt) / 1000).toFixed(1)} s`);
      results.push({ Device: t.row.deviceNumber, Joint: t.row.label, Status: `LOADED (${t.configFile.parameters.length} parameter(s), ${t.dropped.length} excluded)` });
    } catch (err: any) {
      console.error(`${progress} ${t.row.label}: FAILED (${err?.message ?? err})`);
      results.push({ Device: t.row.deviceNumber, Joint: t.row.label, Status: `FAILED (${err?.message ?? err})` });
    }
  }

  console.log("\n=== Load configs summary ===");
  console.table(results);
  const failed = results.filter((r) => !r.Status.startsWith("LOADED"));
  if (failed.length > 0) throw new Error(`${failed.length} config(s) were not loaded — see the table.`);
  return true;
}

// ---------------------------------------------------------------------------------------------
// Stage 3: polarity from the config files
// ---------------------------------------------------------------------------------------------

/**
 * Stage 3: prompts for confirmation, then processes every row with a resolved polarity
 * comparison (status MATCH or MISMATCH):
 *   - MISMATCH: writes 0x200D:00 to match the config file (a plain SDO download to the
 *     device's live parameter set), verifies it, then cycles the controlword. The write is saved
 *     in Stage 4 together with the Encoder 2 single turn offset.
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

  logStage(3, "Apply system polarity from the config files");

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
    "\nMismatching devices get 0x200D:00 written to their live parameter set; it is saved in Stage 4 with the\n" +
      "Encoder 2 single turn offset. Every listed device then has its controlword (0x6040:00) cycled 0 -> 6 -> 7.\n",
  );

  const confirmed = await confirm(`Type "yes" to proceed on these ${eligible.length} device(s), anything else to abort: `);
  if (!confirmed) {
    console.log("\nAborted — no changes applied.\n");
    return;
  }

  await checkStoInactiveOnAllDrives(eligible);

  const results: { Device: string; Joint: string; Status: string }[] = [];
  for (const [i, row] of eligible.entries()) {
    const liveDevice = row.liveDevice!;
    console.log(
      `[${i + 1}/${eligible.length}] ${row.label}: ` +
        (row.status === "MISMATCH" ? `writing polarity ${row.filePolarityValue}, then controlword 0 -> 6 -> 7...` : "controlword 0 -> 6 -> 7..."),
    );
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
      } finally {
        // The transition ends in Switched on: leave the drive in Switch on disabled again.
        try {
          await client.request.download(liveDevice.deviceRef, CONTROLWORD_INDEX, CONTROLWORD_SUBINDEX, CW_DISABLE_VOLTAGE, undefined, requestTimeout);
        } catch {
          console.error(`  [ERROR] could not return ${row.label} to Switched on disabled — check the drive.`);
        }
      }
    } catch (err: any) {
      results.push({ Device: row.deviceNumber, Joint: row.label, Status: `WRITE FAILED (${err?.message ?? err})` });
    }
  }

  console.log("=== Apply summary ===");
  console.table(results);

  const stage2StatusByDevice = new Map(results.map((r) => [r.Device, r.Status]));
  const homed = await centreEncodersAndHome(eligible, stage2StatusByDevice);
  if (!homed) return;
  await saveAndReportPositions(homed);
  await applySoftwarePositionLimits(eligible);
  await applyQuickStopAndSs1Configuration(eligible);
}

// ---------------------------------------------------------------------------------------------
// Stage 4: Encoder 2 single turn offset, then homing
// ---------------------------------------------------------------------------------------------

/** Wraps `n` into [0, ENCODER2_RESOLUTION). */
const wrapEncoder = (n: number) => ((n % ENCODER2_RESOLUTION) + ENCODER2_RESOLUTION) % ENCODER2_RESOLUTION;

/** Shortest signed distance from b to a on the circle of ENCODER2_RESOLUTION counts. */
function circularDiff(a: number, b: number): number {
  const d = wrapEncoder(a - b);
  return d > ENCODER2_HALF ? d - ENCODER2_RESOLUTION : d;
}

/**
 * The single turn offset that puts the position in the middle of the range, from the Encoder 2
 * adjusted position (0x2113:02) read while the offset (0x2112:06) is 0.
 */
function calculateSingleTurnOffset(adjustedAtZeroOffset: number): number {
  return adjustedAtZeroOffset < ENCODER2_HALF ? adjustedAtZeroOffset + ENCODER2_HALF : adjustedAtZeroOffset - ENCODER2_HALF;
}

/**
 * Mean of samples on the circle, so a joint sitting on the 0 / resolution seam does not average
 * to the middle of the range; also returns the largest distance from the mean.
 */
function circularMean(values: number[]): { mean: number; spread: number } {
  const reference = values[0];
  const meanDiff = values.reduce((sum, v) => sum + circularDiff(v, reference), 0) / values.length;
  const mean = wrapEncoder(Math.round(reference + meanDiff));
  const spread = Math.max(...values.map((v) => Math.abs(circularDiff(v, mean))));
  return { mean, spread };
}

async function readEncoderSamples(d: LiveDevice, count: number) {
  const adjusted: number[] = [];
  for (let i = 0; i < count; i++) {
    adjusted.push(await readObject(d, ENCODER2_FEEDBACK_INDEX, ENCODER2_ADJUSTED_SUBINDEX));
    if (i < count - 1) await resolveAfter(ENCODER_SAMPLE_INTERVAL_MS);
  }
  return { adjusted: circularMean(adjusted) };
}

interface CentrePlan {
  row: Row;
  device: LiveDevice;
  adjustedBefore: number;
  currentOffset: number;
  restoreHomeBefore: number;
  homeOffsetBefore: number;
  skip?: string;
}

async function planCentering(row: Row, stage2Status: string | undefined): Promise<CentrePlan> {
  const device = row.liveDevice!;
  const blank = { row, device, adjustedBefore: NaN, currentOffset: NaN, restoreHomeBefore: NaN, homeOffsetBefore: NaN };
  if (stage2Status !== "APPLIED" && stage2Status !== "TRANSITIONED") {
    return { ...blank, skip: `STAGE 2 DID NOT COMPLETE (${stage2Status ?? "no result"})` };
  }
  try {
    const statusword = await readObject(device, STATUSWORD_INDEX, 0);
    if (statusword & SW_OPERATION_ENABLED) return { ...blank, skip: "OPERATION ENABLED (drive is holding the joint)" };
    if (statusword & SW_FAULT) return { ...blank, skip: `FAULT (statusword 0x${statusword.toString(16)})` };

    const currentOffset = await readObject(device, ENCODER2_ST_OFFSET_INDEX, ENCODER2_ST_OFFSET_SUBINDEX);
    const restoreHomeBefore = await readObject(device, RESTORE_HOME_POSITION_INDEX, RESTORE_HOME_POSITION_SUBINDEX);
    const homeOffsetBefore = await readObject(device, HOME_OFFSET_INDEX, HOME_OFFSET_SUBINDEX);
    const { adjusted } = await readEncoderSamples(device, encoderSamples);
    if (adjusted.spread > maxSpread) {
      return { ...blank, currentOffset, restoreHomeBefore, homeOffsetBefore, skip: `JOINT MOVING (adjusted spread ${adjusted.spread} > ${maxSpread})` };
    }
    return { row, device, adjustedBefore: adjusted.mean, currentOffset, restoreHomeBefore, homeOffsetBefore };
  } catch (err: any) {
    return { ...blank, skip: `READ FAILED (${err?.message ?? err})` };
  }
}

/** Writes the previous Encoder 2 single turn offset and Restore home position back and saves. */
async function rollbackCentering(p: CentrePlan, reason: string): Promise<{ ok: boolean; status: string }> {
  const d = p.device;
  try {
    await writeAndVerify(d, ENCODER2_ST_OFFSET_INDEX, ENCODER2_ST_OFFSET_SUBINDEX, p.currentOffset);
    await writeAndVerify(d, RESTORE_HOME_POSITION_INDEX, RESTORE_HOME_POSITION_SUBINDEX, p.restoreHomeBefore);
    await lastValueFrom(client.request.saveConfig(d.deviceRef));
    return { ok: false, status: `${reason}, ROLLED BACK` };
  } catch (err: any) {
    return {
      ok: false,
      status: `${reason}, ROLLBACK FAILED (${err?.message ?? err}) — restore 0x2112:06=${p.currentOffset}, 0x2005:02=${p.restoreHomeBefore} manually`,
    };
  }
}

/**
 * Centres the Encoder 2 single turn range, using only the adjusted position (0x2113:02):
 *   1. Write the single turn offset (0x2112:06) = 0 and read 0x2113:02 (--samples samples).
 *   2. offset = adjusted + 2^19 if adjusted < 2^19, else adjusted - 2^19.
 *   3. Write the offset and Restore home position (0x2005:02) = 1, save, read 0x2113:02 back: it
 *      must be 2^19 within --tolerance counts.
 * On any failure after the first write the previous offset and restore flag are written back
 * and saved.
 */
async function applyCentering(p: CentrePlan): Promise<{ ok: boolean; status: string }> {
  const d = p.device;
  let newOffset: number;
  try {
    await writeAndVerify(d, ENCODER2_ST_OFFSET_INDEX, ENCODER2_ST_OFFSET_SUBINDEX, 0);
    await resolveAfter(ENCODER_SAMPLE_INTERVAL_MS);
    const { adjusted } = await readEncoderSamples(d, encoderSamples);
    if (adjusted.spread > maxSpread) {
      return await rollbackCentering(p, `JOINT MOVING (adjusted spread ${adjusted.spread} > ${maxSpread} with the offset at 0)`);
    }
    newOffset = calculateSingleTurnOffset(adjusted.mean);
    console.log(`    adjusted position with offset 0: ${adjusted.mean} -> single turn offset ${newOffset}`);
  } catch (err: any) {
    return await rollbackCentering(p, `ST OFFSET ZERO/READ FAILED (${err?.message ?? err})`);
  }

  try {
    await writeAndVerify(d, ENCODER2_ST_OFFSET_INDEX, ENCODER2_ST_OFFSET_SUBINDEX, newOffset);
    await writeAndVerify(d, RESTORE_HOME_POSITION_INDEX, RESTORE_HOME_POSITION_SUBINDEX, 1);
    await lastValueFrom(client.request.saveConfig(d.deviceRef));
  } catch (err: any) {
    return { ok: false, status: `ST OFFSET WRITE/SAVE FAILED (${err?.message ?? err}) — check the drive` };
  }

  try {
    const { adjusted } = await readEncoderSamples(d, 5);
    const error = circularDiff(adjusted.mean, ENCODER2_HALF);
    if (Math.abs(error) <= centreTolerance && adjusted.spread <= maxSpread) {
      return { ok: true, status: `ST OFFSET SET to ${newOffset} (adjusted position ${adjusted.mean}, ${error >= 0 ? "+" : ""}${error} from the middle)` };
    }
    return await rollbackCentering(p, `ST OFFSET FAILED (adjusted position ${adjusted.mean}, ${error} from the middle)`);
  } catch (err: any) {
    return { ok: false, status: `ST OFFSET VERIFY FAILED (${err?.message ?? err})` };
  }
}

/** Fails loudly when the Home Offset object (0x607C:00) is not 0. */
function homeOffsetFailure(label: string, homeOffset: number, when: string): string {
  const message = `HOME OFFSET (0x607C:00) IS ${homeOffset} ${when}, IT MUST BE 0`;
  banner([`${label}: ${message}`, "The joint must be homed with a zero Home Offset - do not use this robot until this is resolved"]);
  return message;
}

/**
 * Homing method 37 (current position) on one drive: Modes of operation 6, Homing method
 * (0x6098:00) 37, controlword 6 -> 7 -> 15, then 31 to start homing and wait for Homing attained.
 * Afterwards the Home Offset (0x607C:00) must still be 0 and the Actual position close to 0.
 * The drive is always returned to Switched on disabled with its previous mode of operation.
 */
async function homeAtCurrentPosition(d: LiveDevice, label: string): Promise<{ ok: boolean; status: string }> {
  let previousMode: number | undefined;
  try {
    previousMode = await readObject(d, MODES_OF_OPERATION_INDEX, 0);

    await writeObject(d, MODES_OF_OPERATION_INDEX, 0, HOMING_MODE);
    await writeObject(d, HOMING_METHOD_INDEX, 0, HOMING_METHOD_CURRENT_POSITION);

    for (const controlword of [CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OPERATION]) {
      await writeObject(d, CONTROLWORD_INDEX, CONTROLWORD_SUBINDEX, controlword);
      await resolveAfter(CONTROLWORD_STEP_DELAY_MS);
    }
    const afterEnable = await readObject(d, STATUSWORD_INDEX, 0);
    if (afterEnable & SW_FAULT || !(afterEnable & SW_OPERATION_ENABLED)) {
      return { ok: false, status: `HOMING NOT STARTED (statusword 0x${afterEnable.toString(16)}: not operation enabled)` };
    }

    await writeObject(d, CONTROLWORD_INDEX, CONTROLWORD_SUBINDEX, CW_START_HOMING);
    const deadline = Date.now() + HOMING_TIMEOUT_MS;
    let statusword = 0;
    while (Date.now() < deadline) {
      statusword = await readObject(d, STATUSWORD_INDEX, 0);
      if (statusword & (SW_HOMING_ATTAINED | SW_HOMING_ERROR | SW_FAULT)) break;
      await resolveAfter(50);
    }
    if (statusword & (SW_HOMING_ERROR | SW_FAULT)) return { ok: false, status: `HOMING ERROR (statusword 0x${statusword.toString(16)})` };
    if (!(statusword & SW_HOMING_ATTAINED)) return { ok: false, status: `HOMING TIMEOUT (statusword 0x${statusword.toString(16)})` };

    const homeOffset = await readObject(d, HOME_OFFSET_INDEX, HOME_OFFSET_SUBINDEX);
    if (homeOffset !== 0) return { ok: false, status: homeOffsetFailure(label, homeOffset, "AFTER HOMING") };

    const degrees = countsToDegrees(await readObject(d, POSITION_ACTUAL_INDEX, POSITION_ACTUAL_SUBINDEX));
    const close = Math.abs(degrees) <= homeToleranceDeg;
    return {
      ok: close,
      status: `${close ? "HOMED" : "HOMED, POSITION NOT NEAR 0"} (actual position ${degrees.toFixed(3)}°, home offset 0)`,
    };
  } catch (err: any) {
    return { ok: false, status: `HOMING FAILED (${err?.message ?? err})` };
  } finally {
    try {
      await writeObject(d, CONTROLWORD_INDEX, CONTROLWORD_SUBINDEX, CW_DISABLE_VOLTAGE);
      if (previousMode !== undefined) await writeObject(d, MODES_OF_OPERATION_INDEX, 0, previousMode);
    } catch {
      console.error(`  [ERROR] could not return ${label} to Switched on disabled — check the drive.`);
    }
  }
}

/**
 * Stage 4: for every joint that completed Stage 3, centres the Encoder 2 single turn range and
 * homes the joint there, one joint after the other:
 *
 *   1. Read --samples (5-10) samples of Encoder 2 feedback Adjusted position (0x2113:02) per
 *      drive (refusing a drive that is moving, operation enabled or in fault) and print the table.
 *   2. After confirmation, per joint: print its assigned name and pause; make sure the Home Offset
 *      (0x607C:00) is 0 (a non-zero value is reset and reported); write the single turn offset
 *      (0x2112:06) = 0, read 0x2113:02, calculate the offset from it (adjusted + 2^19 if below
 *      2^19, else adjusted - 2^19), write it and Restore home position (0x2005:02) = 1, save, and
 *      read 0x2113:02 back — it must be 2^19; then Homing method 37 at the current position.
 *   3. At the end of the homing the Home Offset must still be 0, otherwise the run fails loudly.
 *
 * Stops at the first joint that fails and throws once the summary is printed, so the later stages
 * never run on a half-commissioned robot. Returns the homed rows, or undefined when aborted.
 */
async function centreEncodersAndHome(eligible: Row[], stage2StatusByDevice: Map<string, string>): Promise<Row[] | undefined> {
  logStage(4, "Set Encoder 2 single turn offset and home at the middle of the range");

  console.log(
    `Every joint must be at its home position (mechanical zero, or the zero you want) before this stage.\n` +
      `Reading ${encoderSamples} sample(s) of the Encoder 2 position from ${eligible.length} drive(s)...\n`,
  );
  const plans: CentrePlan[] = [];
  for (const row of eligible) plans.push(await planCentering(row, stage2StatusByDevice.get(row.deviceNumber)));

  const show = (n: number) => (Number.isNaN(n) ? "-" : n);
  console.table(
    plans.map((p) => ({
      Device: p.row.deviceNumber,
      Joint: p.row.label,
      "Adjusted before (0x2113:02)": show(p.adjustedBefore),
      "ST offset now (0x2112:06)": show(p.currentOffset),
      "Home offset now (0x607C:00)": show(p.homeOffsetBefore),
      Status: p.skip ?? "READY",
    })),
  );
  for (const p of plans) {
    if (p.skip) continue;
    if (p.homeOffsetBefore !== 0) {
      console.warn(`[WARN] ${p.row.label}: Home Offset (0x607C:00) is ${p.homeOffsetBefore}, not 0 — it will be reset to 0 before homing.`);
    }
  }

  const skipped = plans.filter((p) => p.skip);
  if (skipped.length > 0) {
    throw new Error(
      `${skipped.length} joint(s) cannot go through Stage 4: ${skipped.map((p) => `${p.row.label} [${p.skip}]`).join("; ")}. ` +
        `Resolve this and re-run — homing only some of the joints would leave the robot half-commissioned.`,
    );
  }

  banner([
    `Stage 4 will ENERGIZE ${plans.length} drive(s), one after the other: Operation enabled, Homing mode`,
    `It saves 0x2112:06 so the Encoder 2 position reads ${ENCODER2_HALF}, then homes with method 37 where the joint is now`,
    "Method 37 is not meant to move the joint - keep clear and keep the approved E-stop/STO within reach",
  ]);
  if (!(await confirm(`Type "yes" to set the offset, save and home these ${plans.length} joint(s) one by one, anything else to abort: `))) {
    console.log("\nAborted — nothing written in Stage 4.\n");
    return undefined;
  }

  const results: { Order: number; Device: string; Joint: string; Status: string }[] = [];
  const homed: Row[] = [];
  let failure: string | undefined;

  for (const [i, p] of plans.entries()) {
    const record = (Status: string) => results.push({ Order: i + 1, Device: p.row.deviceNumber, Joint: p.row.label, Status });

    console.log(`\n>>> [${i + 1}/${plans.length}] ${p.row.label} <<<`);
    await resolveAfter(pauseMs);

    try {
      // The Home Offset must be 0 before homing: reset a leftover value (e.g. from a loaded config).
      if (p.homeOffsetBefore !== 0) {
        await writeObject(p.device, HOME_OFFSET_INDEX, HOME_OFFSET_SUBINDEX, 0);
        const verified = await readObject(p.device, HOME_OFFSET_INDEX, HOME_OFFSET_SUBINDEX);
        if (verified !== 0) {
          record(`HOME OFFSET RESET FAILED (read back ${verified})`);
          failure = homeOffsetFailure(p.row.label, verified, "AFTER A RESET");
          break;
        }
      }

      const centred = await applyCentering(p);
      console.log(`    ${centred.status}`);
      if (!centred.ok) {
        record(centred.status);
        failure = `${p.row.label}: ${centred.status}`;
        break;
      }

      const homing = await homeAtCurrentPosition(p.device, p.row.label);
      console.log(`    ${homing.status}`);
      record(`${centred.status}; ${homing.status}`);
      if (!homing.ok) {
        failure = `${p.row.label}: ${homing.status}`;
        break;
      }
      homed.push(p.row);
    } catch (err: any) {
      record(`FAILED (${err?.message ?? err})`);
      failure = `${p.row.label}: ${err?.message ?? err}`;
      break;
    }
  }

  console.log("\n=== Stage 4 summary ===");
  console.table(results);
  if (failure) {
    throw new Error(
      `Stage 4 failed at ${failure}. ${plans.length - results.length} joint(s) were not processed; nothing after Stage 4 was run.`,
    );
  }
  return homed;
}

// ---------------------------------------------------------------------------------------------
// Stage 5: save and report
// ---------------------------------------------------------------------------------------------

/**
 * Stage 5: asks for confirmation, then SAVES all parameters on every homed device (0x1010: Store
 * parameters) so the homed position (Restore home position = 1) and everything written in Stages
 * 2 and 3 persist across a power cycle, and reports each device's Actual position (0x6064:00) and
 * Home Offset (0x607C:00). A Home Offset that is not 0 fails loudly.
 */
async function saveAndReportPositions(homed: Row[]): Promise<void> {
  logStage(5, "Save parameters and report actual position");

  if (homed.length === 0) {
    console.log("No homed devices — nothing to save.\n");
    return;
  }

  console.log(
    `ALL parameters on ${homed.length} homed device(s) will be SAVED (0x1010: Store parameters), so the homed position,\n` +
      `the Encoder 2 single turn offset and the polarity persist across a power cycle.\n`,
  );
  const confirmed = await confirm(`Type "yes" to save these ${homed.length} device(s), anything else to skip: `);

  const results: { Device: string; Joint: string; "Position (deg)": string; "Home Offset": number | string; Status: string }[] = [];
  let failure: string | undefined;
  for (const [i, row] of homed.entries()) {
    const d = row.liveDevice!;
    console.log(`[${i + 1}/${homed.length}] ${confirmed ? "Saving" : "Reading"} ${row.label}...`);
    let status = confirmed ? "SAVED" : "NOT SAVED";
    try {
      if (confirmed) {
        // The homed position only persists with Restore home position when loading configuration = 1 (YES).
        if ((await readObject(d, RESTORE_HOME_POSITION_INDEX, RESTORE_HOME_POSITION_SUBINDEX)) !== 1) {
          await writeObject(d, RESTORE_HOME_POSITION_INDEX, RESTORE_HOME_POSITION_SUBINDEX, 1);
        }
        const restore = await readObject(d, RESTORE_HOME_POSITION_INDEX, RESTORE_HOME_POSITION_SUBINDEX);
        if (restore !== 1) throw new Error(`Restore home position (0x2005:02) reads ${restore}, it must be 1 to persist the homed position`);
        await lastValueFrom(client.request.saveConfig(d.deviceRef));
      }
      const degrees = countsToDegrees(await readObject(d, POSITION_ACTUAL_INDEX, POSITION_ACTUAL_SUBINDEX));
      const homeOffset = await readObject(d, HOME_OFFSET_INDEX, HOME_OFFSET_SUBINDEX);
      if (homeOffset !== 0) {
        status = homeOffsetFailure(row.label, homeOffset, "AFTER HOMING AND SAVING");
        failure = failure ?? `${row.label}: ${status}`;
      }
      results.push({ Device: row.deviceNumber, Joint: row.label, "Position (deg)": degrees.toFixed(3), "Home Offset": homeOffset, Status: status });
    } catch (err: any) {
      status = `FAILED (${err?.message ?? err})`;
      failure = failure ?? `${row.label}: ${status}`;
      results.push({ Device: row.deviceNumber, Joint: row.label, "Position (deg)": "-", "Home Offset": "-", Status: status });
    }
  }
  console.table(results);
  if (failure) throw new Error(`Stage 5 failed at ${failure}.`);
}

/**
 * Stage 6: applies the software position limits (0x607D:01 min, 0x607D:02 max) from each
 * device's matched config file to that live device, for every device that went through Stage 3
 * (both MATCH and MISMATCH) whose config file has both values. Asks for confirmation before
 * writing anything. This write is NOT saved/persisted by this stage (no 0x1010 store
 * parameters) — it only lasts until the next power cycle or config reload, unless the device
 * was separately saved in Stage 5. Prints a summary table converted to degrees, assuming a
 * 20-bit position encoder.
 */
async function applySoftwarePositionLimits(eligible: Row[]): Promise<void> {
  logStage(6, "Apply software position limits");

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
 * went through Stage 3 (both MATCH and MISMATCH):
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

/** Pairs every config file with the live device sharing its joint number and compares the polarity. */
async function buildRows(
  sourceConfigsByNumber: Map<string, SourceConfig[]>,
  deviceNumbers: string[],
): Promise<{ rows: Row[]; liveDevicesByNumber: Map<string, LiveDevice[]> }> {
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
      filePath: config.filePath,
      fileMinPositionLimitCounts: config.fileMinPositionLimitCounts,
      fileMaxPositionLimitCounts: config.fileMaxPositionLimitCounts,
    });
  }

  return { rows, liveDevicesByNumber };
}

function printDiagnosis(
  rows: Row[],
  liveDevicesByNumber: Map<string, LiveDevice[]>,
  sourceConfigsByNumber: Map<string, SourceConfig[]>,
): void {
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
  let { rows, liveDevicesByNumber } = await buildRows(sourceConfigsByNumber, deviceNumbers);
  printDiagnosis(rows, liveDevicesByNumber, sourceConfigsByNumber);

  if (apply) {
    const namesAssigned = await assignNames();
    if (namesAssigned) {
      logStage(0, "Diagnose system polarity again, now that the names are assigned");
      ({ rows, liveDevicesByNumber } = await buildRows(sourceConfigsByNumber, deviceNumbers));
      printDiagnosis(rows, liveDevicesByNumber, sourceConfigsByNumber);
    }
    const configsLoaded = await loadConfigs(rows);
    if (configsLoaded) {
      logStage(0, "Diagnose system polarity again, now that the configs are loaded");
      ({ rows, liveDevicesByNumber } = await buildRows(sourceConfigsByNumber, deviceNumbers));
      printDiagnosis(rows, liveDevicesByNumber, sourceConfigsByNumber);
    }
    await applyPolarity(rows);
  }
}

main()
  .catch((err: any) => {
    console.error(`[FATAL] ${err?.message ?? err}`);
    process.exit(1);
  })
  .finally(() => client.closeSockets());

/**
 * Step 1 of the bring-up: assigns the joint label ("<number> <joint name>", e.g. "104 head_yaw")
 * to every drive on a robot, in a single command, by EtherCAT slave position.
 *
 * The label is the object 0x20F2:00. `write-drive-config.ts` and `load-drive-configs.ts` match
 * every live drive to its config file by the number at the start of that label, so a robot whose
 * drives have never been labelled — a new build, replacement drives — has to be labelled first.
 *
 * Each file in `drive_configs/` is named after the slave position it was exported from
 * (`position_01_head-yaw_config.csv` -> slave position 1) and carries its joint label in its own
 * 0x20F2:00 row. This script copies that label to the drive in the same place on the connected
 * robot.
 *
 * The files are generated with the IgH master, which counts from 0 and counts every slave on
 * the bus, including ones that are not drives (e.g. Beckhoff EtherCAT switches). Such slaves
 * have no file, so the numbers in `drive_configs/` have gaps. The Motion Master here runs SOEM,
 * which only reports the drives. Pass the IgH positions of the slaves that are not drives with
 * --ignore_ecat_indices; the file position is converted to a drive index by taking out the
 * ignored positions that come before it. For Roger-3, where 0, 3 and 18 are switches:
 *
 *   --ignore_ecat_indices 0,3,18
 *   position_01_*.csv  ->  1st drive  (position 0 is before it)
 *   position_02_*.csv  ->  2nd drive
 *   position_04_*.csv  ->  3rd drive  (positions 0 and 3 are before it)
 *   position_19_*.csv  ->  17th drive (positions 0, 3 and 18 are before it)
 *   ...
 *
 * Without the flag nothing is ignored and position 0 is the 1st drive. "Nth drive" means the Nth
 * device in bus order (devices are sorted by position, so SOEM's 1-based and IgH's 0-based
 * numbering make no difference). The step only makes sense when the cabling is monotonic, i.e.
 * the drives are wired in the same order as on the robot the files came from.
 *
 * After each write the label is read back; once it matches, parameters are saved on that drive
 * (0x1010: Store parameters) so the label survives a power cycle. Nothing is written without
 * typing "yes", and --dry-run prints the pairing table and exits without writing.
 *
 * Usage:
 *   # Read-only: show which drive would get which label
 *   npx ts-node src/humanoid_bringup/assign-names.ts --ignore_ecat_indices 0,3,18 --dry-run
 *
 *   # Assign every label (asks for confirmation first)
 *   npx ts-node src/humanoid_bringup/assign-names.ts --ignore_ecat_indices 0,3,18
 *
 *   # One file position only (the IgH position from the filename)
 *   npx ts-node src/humanoid_bringup/assign-names.ts --slave-position 7
 *
 *   # Roger-3: IgH positions 0, 3 and 18 are EtherCAT switches, not drives
 *   npx ts-node src/humanoid_bringup/assign-names.ts --ignore_ecat_indices 0,3,18
 *
 * Next step: `load-drive-configs.ts --dry-run`, then `write-drive-config.ts --diagnose`.
 */

import { readdirSync, readFileSync } from "fs";
import { join } from "path";
import { createInterface } from "readline";

import { Option, program } from "commander";
import { ConfigFile } from "motion-master-client";
import { lastValueFrom } from "rxjs";

import { client } from "../init-client";

const DEVICE_LABEL_INDEX = 0x20f2;
const DEVICE_LABEL_SUBINDEX = 0;
const DEFAULT_CONFIG_DIR = join(__dirname, "drive_configs");

function parseIgnoredIndices(raw: string): number[] {
  return raw
    .split(",")
    .map((t) => t.trim())
    .filter((t) => t.length > 0)
    .map((t) => {
      const n = parseInt(t, 10);
      if (!Number.isInteger(n) || n < 0) throw new Error(`Invalid --ignore_ecat_indices entry "${t}".`);
      return n;
    });
}

program
  .addOption(new Option("--config-dir <path>", "directory holding the joint config files").default(DEFAULT_CONFIG_DIR))
  .addOption(new Option("--slave-position <n>", "only assign the label for this file position (IgH, from the filename)").argParser((v) => {
    const n = parseInt(v, 10);
    if (!Number.isInteger(n) || n < 0) throw new Error(`Invalid --slave-position "${v}". Expected a non-negative integer.`);
    return n;
  }))
  .addOption(
    new Option(
      "--ignore_ecat_indices <list>",
      "comma-separated IgH positions of slaves that are not drives (e.g. EtherCAT switches) and so have no config file, e.g. 0,3,18",
    )
      .default([] as number[], "none")
      .argParser(parseIgnoredIndices),
  )
  .addOption(new Option("--dry-run", "show the pairing table without writing anything").default(false));

program.allowExcessArguments(false);
program.showHelpAfterError("(run with --help to see the available options)");
program.parse();

const { configDir, slavePosition, ignore_ecat_indices: ignoredIndices, dryRun, requestTimeout = 10000 } = program.opts<{
  configDir: string;
  slavePosition?: number;
  ignore_ecat_indices: number[];
  dryRun: boolean;
  requestTimeout?: number;
}>();

/** Prompts on stdin and resolves true only if the answer is "yes" (case-insensitive). */
function confirm(promptText: string): Promise<boolean> {
  const rl = createInterface({ input: process.stdin, output: process.stdout });
  return new Promise((resolve) => {
    rl.question(promptText, (answer) => {
      rl.close();
      resolve(answer.trim().toLowerCase() === "yes");
    });
  });
}

function printWarning(): void {
  const lines = [
    "Applying this step assumes EtherCAT network is wired exactly same between the robots",
    ignoredIndices.length > 0
      ? `Assuming monotonic cabling and position ${ignoredIndices.join(", ")} as EtherCAT switch positions`
      : "Assuming monotonic cabling and no ignored EtherCAT positions (see --ignore_ecat_indices)",
  ];
  const inner = Math.max(...lines.map((l) => l.length)) + 4;
  const bar = "!".repeat(inner + 4);
  const blank = `!!${" ".repeat(inner)}!!`;
  const body = lines.map((l) => `!!  ${l.padEnd(inner - 2)}!!`);
  console.warn(`\n${[bar, bar, blank, ...body, blank, bar, bar].join("\n")}\n`);
}

/** The IgH position of a file -> the 0-based index of its drive in bus order, with the ignored positions taken out. */
function driveIndexOf(igHPosition: number): number {
  return igHPosition - ignoredIndices.filter((s) => s < igHPosition).length;
}

interface NameSource {
  slavePosition: number;
  fileName: string;
  /** The "<number> <joint name>" string from the file's 0x20F2:00 row. */
  label: string;
}

/** Reads every `position_NN_*.csv` in `dir` and returns the label each one carries, by slave position. */
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
    if (!/^\d+\b/.test(label)) {
      problems.push(`${fileName}: no usable 0x20F2:00 label ("<number> <joint name>") in the file, ignored`);
      continue;
    }

    const existing = sources.get(position);
    if (existing) {
      problems.push(`slave position ${position}: claimed by both ${existing.fileName} and ${fileName}`);
      continue;
    }
    sources.set(position, { slavePosition: position, fileName, label });
  }

  return { sources, problems };
}

interface Row {
  /** The IgH position from the filename. */
  slavePosition: number;
  /** 1-based index of the drive in bus order, with the ignored positions taken out. */
  driveNumber?: number;
  deviceRef?: string | number;
  deviceAddress?: number;
  currentLabel: string;
  newLabel: string;
  fileName: string;
  /** Set when the row cannot or should not be written. */
  skip?: string;
}

async function main(): Promise<void> {
  console.log(`\n=== Step 1: Assign names by slave position${dryRun ? " (DRY RUN)" : ""} ===`);
  printWarning();

  const { sources, problems } = scanNameSources(configDir);
  console.log(`Config directory: ${configDir}`);
  console.log(`Found ${sources.size} config file(s) with a slave position and a label.`);
  for (const p of problems) console.warn(`  [WARN] ${p}`);
  if (sources.size === 0) {
    throw new Error(`No usable position_<NN>_*.csv files in ${configDir}.`);
  }

  const devices = await lastValueFrom(client.request.getDevices(requestTimeout));
  const byBusOrder = [...devices].sort((a, b) => (a.position ?? 0) - (b.position ?? 0));
  console.log(`Discovered ${byBusOrder.length} drive(s) on the network.\n`);

  if (byBusOrder.length !== sources.size) {
    console.warn(
      `[WARN] The network has ${byBusOrder.length} drive(s) but there are ${sources.size} config file(s). ` +
        `The robot is probably not the one the files came from.\n`,
    );
  }

  const rows: Row[] = [];
  for (const source of [...sources.values()].sort((a, b) => a.slavePosition - b.slavePosition)) {
    if (slavePosition !== undefined && source.slavePosition !== slavePosition) continue;

    if (ignoredIndices.includes(source.slavePosition)) {
      rows.push({
        slavePosition: source.slavePosition,
        currentLabel: "-",
        newLabel: source.label,
        fileName: source.fileName,
        skip: "FILE AT AN IGNORED POSITION",
      });
      continue;
    }

    const driveIndex = driveIndexOf(source.slavePosition);
    const device = byBusOrder[driveIndex];
    if (!device) {
      rows.push({
        slavePosition: source.slavePosition,
        driveNumber: driveIndex + 1,
        currentLabel: "-",
        newLabel: source.label,
        fileName: source.fileName,
        skip: "NO DRIVE AT THIS POSITION",
      });
      continue;
    }

    const deviceRef = device.serialNumber ?? device.deviceAddress;
    let currentLabel = "(unreadable)";
    try {
      currentLabel = String(
        await client.request.upload<string>(deviceRef, DEVICE_LABEL_INDEX, DEVICE_LABEL_SUBINDEX, false, requestTimeout),
      );
    } catch {
      // A drive that was never labelled, or a slave without 0x20F2:00 — the write below decides.
    }
    rows.push({
      slavePosition: source.slavePosition,
      driveNumber: driveIndex + 1,
      deviceRef,
      deviceAddress: device.deviceAddress,
      currentLabel,
      newLabel: source.label,
      fileName: source.fileName,
    });
  }

  if (slavePosition !== undefined && rows.length === 0) {
    throw new Error(`--slave-position ${slavePosition} has no config file in ${configDir}.`);
  }

  // Two files with the same joint number would make every later step ambiguous.
  const numberOf = (label: string) => label.match(/^(\d+)\b/)?.[1] ?? label;
  const seen = new Map<string, Row>();
  for (const row of rows) {
    const key = numberOf(row.newLabel);
    const other = seen.get(key);
    if (other) {
      row.skip = row.skip ?? `DUPLICATE JOINT NUMBER (also file position ${other.slavePosition})`;
    } else {
      seen.set(key, row);
    }
  }

  console.table(
    rows.map((r) => ({
      "File position (IgH)": r.slavePosition,
      "Drive # (SOEM)": r.driveNumber ?? "-",
      "Device address": r.deviceAddress !== undefined ? `0x${r.deviceAddress.toString(16).toUpperCase()}` : "-",
      "Current label": r.currentLabel,
      "New label": r.newLabel,
      File: r.fileName,
      Status: r.skip ?? (r.currentLabel === r.newLabel ? "ALREADY SET" : dryRun ? "WOULD ASSIGN" : "PENDING"),
    })),
  );

  const mapped = new Set(rows.map((r) => r.driveNumber).filter((n): n is number => n !== undefined));
  const untouched = byBusOrder.map((_, i) => i + 1).filter((n) => !mapped.has(n));
  if (slavePosition === undefined && untouched.length > 0) {
    console.log(`Drive(s) with no config file, left untouched: ${untouched.join(", ")}\n`);
  }

  if (dryRun) {
    console.log("Dry run — nothing written.\n");
    return;
  }

  const toWrite = rows.filter((r) => !r.skip && r.currentLabel !== r.newLabel);
  if (toWrite.length === 0) {
    console.log("Nothing to assign.\n");
    return;
  }

  printWarning();
  console.log(
    `${toWrite.length} label(s) will be written to 0x20F2:00 and saved (0x1010: Store parameters) on the drive\n` +
      `at each slave position above. Check the table: a drive at the wrong position gets the wrong name.\n`,
  );
  const confirmed = await confirm(`Type "yes" to assign these ${toWrite.length} name(s), anything else to abort: `);
  if (!confirmed) {
    console.log("\nAborted — no names assigned.\n");
    return;
  }

  const results: { "File position (IgH)": number; "New label": string; Status: string }[] = [];
  for (const row of toWrite) {
    const result = (Status: string) =>
      results.push({ "File position (IgH)": row.slavePosition, "New label": row.newLabel, Status });
    try {
      await client.request.download(
        row.deviceRef!,
        DEVICE_LABEL_INDEX,
        DEVICE_LABEL_SUBINDEX,
        row.newLabel,
        "stringValue",
        requestTimeout,
      );
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
      } catch (err: unknown) {
        result(`ASSIGNED, SAVE FAILED (${err instanceof Error ? err.message : err})`);
      }
    } catch (err: unknown) {
      result(`WRITE FAILED (${err instanceof Error ? err.message : err})`);
    }
  }

  console.log("\n=== Assign names summary ===");
  console.table(results);
  console.log("Next: load-drive-configs.ts --dry-run, then write-drive-config.ts --diagnose.\n");
}

client
  .whenReady()
  .then(main)
  .catch((err: unknown) => {
    console.error(`[FATAL] ${err instanceof Error ? err.message : err}`);
    process.exitCode = 1;
  })
  .finally(() => client.closeSockets());

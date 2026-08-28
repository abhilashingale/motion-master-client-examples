/**
 * Loads the joint configuration files onto their drives, leaving chosen objects out.
 *
 * Devices and config files are matched the same way `write-drive-config.ts` matches them, so
 * both scripts agree about which file belongs to which joint: every config file and every live
 * device carry a "<number> <joint name>" label at 0x20F2:00 (e.g. "104 head_yaw"), and the
 * leading number is the key. If a file has no 0x20F2:00 row, its filename is used instead
 * (`104_head_pitch.config.csv` -> 104), so a file that has not been given a label yet still
 * matches; which source was used is printed per joint.
 *
 * Without --device-ref, every live device that exposes a label is matched to its config file
 * and loaded. With --device-ref, only that device is loaded, still using its own matched file.
 *
 * --exclude applies to every joint. It is meant for values that belong to one physical drive
 * rather than to the machine configuration — the commutation angle offset, encoder offsets, the
 * station alias — so a shared config does not overwrite what was calibrated on the drive.
 *
 * The strategy decides what excluding actually means:
 *
 *   --strategy merge     (default here)
 *     Motion Master reads the device's existing config.csv, applies the file's rows on top, and
 *     writes the result back. An excluded object keeps the value the device already has.
 *
 *   --strategy replace   (the library's default)
 *     config.csv is overwritten with exactly the rows sent. An excluded object is then absent
 *     from the file, so on 0x1011:01 Restore All Default Parameters the firmware falls back to
 *     .factory_config / .assembly_config for it — excluding REVERTS it rather than preserving
 *     it. Only use this when that is what you mean.
 *
 * The joint label at 0x20F2:00 is itself excluded by default: loading it is harmless when the
 * file matched that device, but it is the one object that must never be written to the wrong
 * drive, and --strategy replace would leave a device unlabelled if its file lacked the row.
 * Pass --include-label to write it.
 *
 * SMM safety parameters are always dropped: the safety layer ignores plain writes to them and
 * only accepts the authenticated, CRC-validated SMM configuration sequence (see
 * `../Vesoma_SS1/configure-damping-parameters.promise.ts`), so sending them here would silently
 * do nothing.
 *
 * Usage:
 *   # Every device, dry run first — read-only, nothing is written
 *   npx ts-node src/humanoid_bringup/load-drive-configs.ts --dry-run
 *
 *   # Every device, keeping each drive's own commutation offset and encoder 2 configuration
 *   npx ts-node src/humanoid_bringup/load-drive-configs.ts --exclude 0x2001:00,0x2112
 *
 *   # One joint only
 *   npx ts-node src/humanoid_bringup/load-drive-configs.ts -d 104 --exclude 0x2001:00
 *
 * ---
 *
 * This writes config.csv on the drives and triggers a configuration load. Run --dry-run first
 * and read the table: a mismatched file would apply one joint's configuration to another.
 */

import { readdirSync, readFileSync } from "fs";
import { join } from "path";

import { Option, program } from "commander";
import { ConfigFile, ConfigParameter, isSmmParameterIndex, LoadConfigStrategy } from "motion-master-client";
import { lastValueFrom } from "rxjs";

import { client } from "../init-client";

const DEVICE_LABEL_INDEX = 0x20f2;
const DEVICE_LABEL_SUBINDEX = 0;
const DEFAULT_CONFIG_DIR = join(__dirname, "drive_configs");

/** `0x2001:00` -> that subindex only; `0x2001` -> every subindex of the object. */
interface ExclusionRule {
  index: number;
  subindex?: number;
  raw: string;
}

function parseExclusions(raw: string): ExclusionRule[] {
  return raw
    .split(",")
    .map((t) => t.trim())
    .filter((t) => t.length > 0)
    .map((token) => {
      const [indexPart, subindexPart] = token.split(":");
      const index = parseInt(indexPart, 16);
      if (isNaN(index)) {
        throw new Error(`Invalid --exclude entry "${token}". Expected e.g. 0x2001:00 or 0x2001.`);
      }
      if (subindexPart === undefined) return { index, raw: token };
      const subindex = parseInt(subindexPart, 10);
      if (isNaN(subindex)) throw new Error(`Invalid subindex in --exclude entry "${token}".`);
      return { index, subindex, raw: token };
    });
}

program
  .addOption(
    new Option(
      "--exclude <ids>",
      'comma-separated objects to leave out of every joint, e.g. "0x2001:00,0x2112" (an entry without a subindex excludes every subindex)',
    ).argParser(parseExclusions),
  )
  .addOption(
    new Option("--strategy <strategy>", "how each device's config.csv is updated")
      .choices(["merge", "replace"])
      .default("merge"),
  )
  .addOption(new Option("--config-dir <path>", "directory holding the joint config files").default(DEFAULT_CONFIG_DIR))
  .addOption(new Option("--include-label", "also write the joint label at 0x20F2:00").default(false))
  .addOption(new Option("--dry-run", "report what would be sent without writing anything").default(false));

program.allowExcessArguments(false);
program.showHelpAfterError("(run with --help to see the available options)");
program.parse();

const {
  deviceRef: requestedDeviceRef,
  exclude = [] as ExclusionRule[],
  strategy,
  configDir,
  includeLabel,
  dryRun,
  requestTimeout = 10000,
} = program.opts<{
  deviceRef?: number | string;
  exclude?: ExclusionRule[];
  strategy: LoadConfigStrategy;
  configDir: string;
  includeLabel: boolean;
  dryRun: boolean;
  requestTimeout?: number;
}>();

/** True when --device-ref was given, rather than left at init-client's default of 0. */
const deviceRefGiven = program.getOptionValueSource("deviceRef") !== "default";

/** Extracts the leading device number from a "<number> <joint name>" label — as write-drive-config.ts does. */
function extractDeviceNumber(label: string): string | undefined {
  const match = label.trim().match(/^(\d+)\b/);
  return match ? match[1] : undefined;
}

const objectId = (p: Pick<ConfigParameter, "index" | "subindex">) =>
  `0x${p.index.toString(16).toUpperCase().padStart(4, "0")}:${String(p.subindex).padStart(2, "0")}`;

interface SourceConfig {
  fileName: string;
  filePath: string;
  /** The 0x20F2:00 label from inside the file, when it has one. */
  label?: string;
  /** Where the device number came from: the label row, or the filename. */
  matchedBy: "0x20F2:00" | "filename";
}

/** Groups every config file in `dir` by device number, preferring the in-file label over the filename. */
function scanSourceConfigs(dir: string): Map<string, SourceConfig[]> {
  const byDeviceNumber = new Map<string, SourceConfig[]>();

  for (const fileName of readdirSync(dir).filter((n) => n.endsWith(".csv"))) {
    const filePath = join(dir, fileName);
    const configFile = new ConfigFile(readFileSync(filePath, "utf-8"));

    const labelParam = configFile.parameters.find(
      (p) => p.index === DEVICE_LABEL_INDEX && p.subindex === DEVICE_LABEL_SUBINDEX,
    );

    let number: string | undefined;
    let matchedBy: SourceConfig["matchedBy"] = "0x20F2:00";
    let label: string | undefined;

    if (labelParam) {
      label = String(labelParam.value);
      number = extractDeviceNumber(label);
    }
    if (!number) {
      // Fall back to the descriptive filename, e.g. "104_head_pitch.config.csv".
      number = extractDeviceNumber(fileName.replace(/[_-]/g, " "));
      matchedBy = "filename";
    }
    if (!number) continue;

    const list = byDeviceNumber.get(number) ?? [];
    list.push({ fileName, filePath, label, matchedBy });
    byDeviceNumber.set(number, list);
  }

  return byDeviceNumber;
}

interface LiveDevice {
  deviceRef: string | number;
  label: string;
  deviceNumber: string;
}

/** Discovers devices and reads the 0x20F2:00 label from each — as write-drive-config.ts does. */
async function scanLiveDevices(): Promise<LiveDevice[]> {
  const devices = await lastValueFrom(client.request.getDevices(requestTimeout));
  console.log(`Discovered ${devices.length} device(s) on the network.`);

  const live: LiveDevice[] = [];
  for (const device of devices) {
    const ref = device.serialNumber ?? device.deviceAddress;
    try {
      const label = String(
        await client.request.upload<string>(ref, DEVICE_LABEL_INDEX, DEVICE_LABEL_SUBINDEX, false, requestTimeout),
      );
      const deviceNumber = extractDeviceNumber(label);
      if (!deviceNumber) continue;
      live.push({ deviceRef: ref, label, deviceNumber });
    } catch {
      // No 0x20F2:00 on this device (e.g. a non-CiA402 device) — it has no joint config.
      continue;
    }
  }
  return live;
}

interface Outcome {
  joint: string;
  file: string;
  matchedBy: string;
  sent: number | string;
  excluded: number | string;
  status: string;
}

/** Filters a config file down to what should be sent, and reports what was dropped. */
function filterForSending(configFile: ConfigFile): { dropped: { p: ConfigParameter; reason: string }[] } {
  const dropped: { p: ConfigParameter; reason: string }[] = [];

  configFile.parameters = configFile.parameters.filter((p) => {
    const rule = exclude.find((r) => r.index === p.index && (r.subindex === undefined || r.subindex === p.subindex));
    if (rule) {
      dropped.push({ p, reason: `--exclude ${rule.raw}` });
      return false;
    }
    if (isSmmParameterIndex(p.index)) {
      dropped.push({ p, reason: "SMM safety parameter" });
      return false;
    }
    if (!includeLabel && p.index === DEVICE_LABEL_INDEX && p.subindex === DEVICE_LABEL_SUBINDEX) {
      dropped.push({ p, reason: "joint label (use --include-label)" });
      return false;
    }
    return true;
  });

  return { dropped };
}

client
  .whenReady()
  .then(async () => {
    console.log(`\n=== Load joint configs (strategy: ${strategy}${dryRun ? ", DRY RUN" : ""}) ===\n`);
    console.log(`Config directory: ${configDir}`);
    if (exclude.length > 0) {
      console.log(`Excluded from every joint: ${exclude.map((r) => r.raw).join(", ")}`);
    } else {
      console.log("Excluded from every joint: (nothing beyond SMM parameters and the joint label)");
    }

    const configsByNumber = scanSourceConfigs(configDir);
    console.log(`Found ${configsByNumber.size} config file(s) with a resolvable joint number.\n`);

    let live = await scanLiveDevices();
    if (deviceRefGiven) {
      const wanted = String(requestedDeviceRef);
      const before = live.length;
      live = live.filter(
        (d) =>
          d.deviceNumber === wanted ||
          String(d.deviceRef) === wanted ||
          d.label.toLowerCase().includes(wanted.toLowerCase()),
      );
      if (live.length === 0) {
        throw new Error(
          `--device-ref ${wanted} matched none of the ${before} labelled device(s). ` +
            `Pass a joint number (e.g. 104), a serial number, or part of the joint name.`,
        );
      }
    }

    console.log(`Loading ${live.length} device(s).\n`);

    const outcomes: Outcome[] = [];

    for (const device of live) {
      const candidates = configsByNumber.get(device.deviceNumber) ?? [];

      if (candidates.length === 0) {
        console.warn(`[SKIP] ${device.label}: no config file for joint number ${device.deviceNumber}.`);
        outcomes.push({ joint: device.label, file: "-", matchedBy: "-", sent: "-", excluded: "-", status: "NO CONFIG FILE" });
        continue;
      }
      if (candidates.length > 1) {
        console.warn(
          `[SKIP] ${device.label}: ${candidates.length} config files claim joint number ${device.deviceNumber} ` +
            `(${candidates.map((c) => c.fileName).join(", ")}). Refusing to guess.`,
        );
        outcomes.push({
          joint: device.label,
          file: candidates.map((c) => c.fileName).join(", "),
          matchedBy: "-",
          sent: "-",
          excluded: "-",
          status: "AMBIGUOUS CONFIG",
        });
        continue;
      }

      const source = candidates[0];
      const configFile = new ConfigFile(readFileSync(source.filePath, "utf-8"));
      const total = configFile.numberOfParameters;
      const { dropped } = filterForSending(configFile);

      console.log(`--- ${device.label}  (device ${device.deviceRef}) ---`);
      console.log(`  file: ${source.fileName}   matched by: ${source.matchedBy}`);
      if (source.label && source.label !== device.label) {
        console.warn(`  [WARN] file label "${source.label}" differs from the device label "${device.label}".`);
      }
      console.log(`  ${configFile.parameters.length} of ${total} parameter(s) to send, ${dropped.length} excluded`);
      for (const { p, reason } of dropped) {
        console.log(`    - ${objectId(p)} = ${p.value}   (${reason})`);
      }

      if (dryRun) {
        outcomes.push({
          joint: device.label,
          file: source.fileName,
          matchedBy: source.matchedBy,
          sent: configFile.parameters.length,
          excluded: dropped.length,
          status: "DRY RUN",
        });
        console.log();
        continue;
      }

      if (configFile.parameters.length === 0) {
        console.error("  [ERROR] every parameter was excluded — refusing to load an empty configuration.\n");
        outcomes.push({ joint: device.label, file: source.fileName, matchedBy: source.matchedBy, sent: 0, excluded: dropped.length, status: "EMPTY" });
        continue;
      }

      try {
        await lastValueFrom(
          client.request.loadConfig(device.deviceRef, configFile.toString(), strategy, { count: 10, delay: 500 }),
        );
        console.log("  loaded.\n");
        outcomes.push({
          joint: device.label,
          file: source.fileName,
          matchedBy: source.matchedBy,
          sent: configFile.parameters.length,
          excluded: dropped.length,
          status: "LOADED",
        });
      } catch (err: unknown) {
        console.error(`  [ERROR] ${err instanceof Error ? err.message : err}\n`);
        outcomes.push({
          joint: device.label,
          file: source.fileName,
          matchedBy: source.matchedBy,
          sent: configFile.parameters.length,
          excluded: dropped.length,
          status: "FAILED",
        });
      }
    }

    // Config files with no live device are worth naming: on a full bring-up a missing joint is
    // usually a drive that did not come up, not an intentional omission.
    const liveNumbers = new Set(live.map((d) => d.deviceNumber));
    const unmatchedFiles = [...configsByNumber.entries()].filter(([number]) => !liveNumbers.has(number));
    if (!deviceRefGiven && unmatchedFiles.length > 0) {
      console.warn(`\n${unmatchedFiles.length} config file(s) had no live device:`);
      for (const [number, files] of unmatchedFiles) {
        console.warn(`  ${number}: ${files.map((f) => f.fileName).join(", ")}`);
      }
    }

    console.log();
    console.table(outcomes);
  })
  .catch((err: unknown) => {
    console.error(`[FATAL] ${err instanceof Error ? err.message : err}`);
    process.exitCode = 1;
  })
  .finally(() => client.closeSockets());

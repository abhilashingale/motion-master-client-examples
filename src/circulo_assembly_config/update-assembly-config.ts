/**
 * Reads `.assembly_config` from every device connected to Motion Master (Circulos) and changes
 * 0x2112:08 from 4167 to 1923.
 *
 * Dry run by default: files are read and backed up, nothing is written. Pass `--apply` to write.
 * A device is modified only if its 0x2112:08 currently equals 4167; otherwise it is skipped.
 *
 * Usage (MOTION_MASTER_HOSTNAME must be set, e.g. in .env):
 *   npx ts-node src/circulo_assembly_config/update-assembly-config.ts [--apply] [--backup-dir <dir>]
 */
import * as fs from 'fs';
import * as path from 'path';
import { lastValueFrom } from 'rxjs';
import { program } from 'commander';
import { client } from '../init-client';

const FILE_NAME = '.assembly_config';
const OLD_VALUE = 4167;
const NEW_VALUE = 1923;

// Matches the row "0x2112, 8, 4167" (whitespace tolerant, leading zeros in the subindex tolerated).
const ROW = /^(\s*0x2112\s*,\s*0*8\s*,\s*)([^\r\n,]*?)(\s*)$/im;

function readValue(content: string): number | undefined {
  const m = content.match(ROW);
  return m ? Number(m[2]) : undefined;
}

async function processDevice(
  device: { id: string; position?: number; serialNumber?: string },
  apply: boolean,
  backupDir: string,
): Promise<void> {
  const label = `position ${device.position} (serial ${device.serialNumber ?? '?'})`;
  const ref = device.id;

  const files = await lastValueFrom(client.request.getFiles(ref));
  if (!files.includes(FILE_NAME)) {
    console.log(`${label}: no ${FILE_NAME} (files: ${files.join(', ') || 'none'}) - skipped`);
    return;
  }

  const content = await lastValueFrom(client.request.getDecodedFile(ref, FILE_NAME));
  fs.mkdirSync(backupDir, { recursive: true });
  const backupPath = path.join(backupDir, `${device.serialNumber ?? `pos${device.position}`}${FILE_NAME}`);
  fs.writeFileSync(backupPath, content);
  console.log(`${label}: read ${FILE_NAME}, backup -> ${backupPath}`);

  const current = readValue(content);
  if (current === undefined) {
    console.log(`${label}: 0x2112:08 not present - skipped`);
    return;
  }
  console.log(`${label}: 0x2112:08 = ${current}`);
  if (current !== OLD_VALUE) {
    console.log(`${label}: value is not ${OLD_VALUE} - skipped`);
    return;
  }

  // Edit only that row so the header and every other line stay byte-identical.
  const updated = content.replace(ROW, `$1${NEW_VALUE}$3`);
  if (updated === content || readValue(updated) !== NEW_VALUE) {
    console.log(`${label}: could not rewrite the row safely - skipped`);
    return;
  }

  if (!apply) {
    console.log(`${label}: DRY RUN - would set 0x2112:08 ${OLD_VALUE} -> ${NEW_VALUE}`);
    return;
  }

  await lastValueFrom(client.request.unlockProtectedFiles(ref));
  await lastValueFrom(client.request.setFile(ref, FILE_NAME, new TextEncoder().encode(updated), true));

  const v = readValue(await lastValueFrom(client.request.getDecodedFile(ref, FILE_NAME)));
  console.log(`${label}: written, read-back 0x2112:08 = ${v} ${v === NEW_VALUE ? 'OK' : 'MISMATCH'}`);
}

program
  .option('--apply', 'write the change (default is a dry run)')
  .option('--backup-dir <dir>', 'where to save copies of the original files', './assembly_config_backup')
  .parse();
const opts = program.opts<{ apply?: boolean; backupDir: string }>();

(async () => {
  await client.whenReady();
  try {
    const devices = await lastValueFrom(client.request.getDevices());
    console.log(`Found ${devices.length} device(s); mode: ${opts.apply ? 'APPLY' : 'dry run'}`);
    for (const device of devices) {
      try {
        await processDevice(device, !!opts.apply, path.resolve(opts.backupDir));
      } catch (err) {
        console.error(`position ${device.position}: failed - ${(err as Error).message ?? err}`);
        process.exitCode = 1;
      }
    }
  } finally {
    client.closeSockets();
  }
})().catch((err) => {
  console.error(err);
  process.exit(1);
});

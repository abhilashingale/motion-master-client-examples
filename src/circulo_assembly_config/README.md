# Circulo `.assembly_config` update

`update-assembly-config.ts` reads `.assembly_config` from every connected device, backs it up, and
changes `0x2112:08` from `4167` to `1923`.

- Dry run by default; pass `--apply` to write. Always dry-run first.
- Only devices whose `0x2112:08` is exactly `4167` are changed; others are skipped and reported.
- Only that one row is edited; the file is read back after writing to verify.
- Edits the file only, not the live object; a restart or config reload may be needed.

```sh
npx ts-node src/circulo_assembly_config/update-assembly-config.ts            # dry run
npx ts-node src/circulo_assembly_config/update-assembly-config.ts --apply    # write
```

Requires `MOTION_MASTER_HOSTNAME` (e.g. in `.env`).

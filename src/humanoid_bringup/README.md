# humanoid_bringup — drive bring-up template

A starting point for bringing up the drives of a multi-joint humanoid: drop your exported
joint configs into `drive_configs/`, and the scripts here pair each live drive with its config
file and act on it.

| Script | What it does |
|---|---|
| `write-drive-config.ts` | The commissioning script, in confirmed stages: diagnoses **system polarity** (`0x200D:00`) against each config file, can assign the joint names by slave position (Stage 1), loads each joint's config file without the Home Offset (Stage 2), applies the polarity (Stage 3), centres the Encoder 2 single-turn range and homes every joint there (Stage 4), saves (Stage 5), and applies software position limits (Stage 6) and quick stop / SS1 safety configuration (Stage 7). |

**This directory ships with `drive_configs/` empty.** Nothing runs until you put config files
in it — see [Using this as a template](#using-this-as-a-template) below.

It is a single script: one `--apply` run does the whole commissioning, stage by stage, asking for
`yes` before each stage that writes. The full sequence is in
[Bring up in this order](#4-bring-up-in-this-order).

Diagnoses whether the **system polarity** (`0x200D:00`) of the live devices on the currently
connected Motion Master session matches the values in their corresponding config files under
`drive_configs/`, and prints a console table plus a summary. By default it checks every
joint; pass `--device-number` to check just one.

`0x200D:00` is `0` for normal polarity or `1` for inverted polarity.

Pass `--diagnose` to run the check (read-only — no parameter is written). Pass `--apply`
instead to also apply changes after you confirm — `--apply` always runs the diagnose step
first. Without either flag, the script does nothing and never connects to Motion Master.

The console output is split into numbered stages so the process is easy to follow:

| Stage | Runs with | What it does |
|-------|-----------|---------------|
| **0** | `--diagnose`, `--apply` | Diagnose the current polarity and compare it against the config files by joint name. Read-only. |
| **1** | `--apply --assign-names` | Write each drive's joint label (`0x20F2:00`) from the `position_<NN>_*.csv` files, by position in the EtherCAT chain, so a robot whose drives were never labelled is named in the same run. `--ignore_ecat_indices` lists the IgH positions that are not drives. Prints a large wiring warning. Stage 0 is then repeated with the new names. Skipped without `--assign-names`. |
| **2** | `--apply` (not with `--skip-load-configs`) | Load each paired config file onto its drive, **without `0x607C` (Home Offset)** — the console says so, per joint, with the value the file holds — and without anything given with `--exclude`. Strategy `merge`. Asks for confirmation; Stage 0 is then repeated. |
| **3** | `--apply` only | Ask for confirmation, then check STO (Safe Torque Off) is inactive on every device — abort with a warning if not. Then write the **system polarity from the config file** (`MISMATCH` devices only) and cycle the controlword `0 -> 6 -> 7` on every `MATCH`/`MISMATCH` device. The polarity is saved in Stage 4. |
| **4** | `--apply` only | Per joint, one after the other, with its assigned name printed first: set **Encoder 2 single turn offset (`0x2112:06`)** so the Encoder 2 adjusted position (`0x2113:02`) is exactly 2^19, the middle of its single-turn range; set Restore home position (`0x2005:02`) = 1; save; read `0x2113:02` back; then **home at the current position with method 37**. No home offset correction is made — the Home Offset (`0x607C:00`) must be `0` at the end of the homing, otherwise the run **fails loudly**. |
| **5** | `--apply` only | Ask for confirmation, then **save all parameters** on every homed device (`0x1010`: Store parameters) so the homed position persists, and print each device's Actual position (`0x6064:00`) and Home Offset (`0x607C:00`). A non-zero Home Offset fails loudly. |
| **6** | `--apply` only | Asks for confirmation, then writes the software position limits (`0x607D:01` min, `0x607D:02` max) from each device's matched config file to that device, for every Stage 3 device whose config file has both values, and — once verified — **saves all parameters** on that device. Prints a summary in degrees. |
| **7** | `--apply` only | "Quick and SS1 Configuration" — the same parameters `Damping Configuration/configure-damping-parameters.promise.ts` writes. Quick stop option code (`0x605A:00`) and ASC duration (`0x2010:12`) are written via plain SDO and saved; three SMM safety parameters (`0x2641:01`, `0x2650:01`, `0x6651:01`) are written via the authenticated SMM configuration flow using `--username`/`--password`. Only devices whose values differ from target are touched. Asks for confirmation first. |

A failure in Stages 1-5 throws and stops the run, so Stages 6 and 7 never run on a
half-commissioned robot.

Stage 3's controlword transition (`0x6040:00`: clear -> Shutdown -> Switch On) makes a
`MISMATCH` device's newly written polarity take effect; `MATCH` devices have nothing written,
but the same transition is still applied.

> **On STO:** before cycling the controlword, Stage 3 reads STO (Safe Torque Off — bit 0 of the
> Safety Statusword's first byte, `0x6621:01`) on every device about to be transitioned. The
> controlword transition `0 -> 6 -> 7` cannot succeed while STO is active (the drive can't leave
> "Switch on disabled"). If STO is active — or unreadable — on *any* device, the entire `--apply`
> run aborts immediately with `[FATAL] STO (Safe Torque Off) is active on: ...`, naming the
> affected device(s), before anything is written. Release STO on those devices and re-run
> `--apply`.

> **On persistence:** Stage 3's polarity write is a plain SDO download, saved by Stage 4's save
> (`0x1010`) together with the Encoder 2 single turn offset and Restore home position. Stage 5
> saves again after the homing, so the homed position persists. Stage 6 saves the software
> position limits and Stage 7 saves its own quick-stop objects (`0x1010`) and persists its SMM
> safety objects via the SMM validation flow.

> **This energizes the drives.** Stage 3 cycles the controlword to Switch on, and Stage 4 enables
> operation for the homing. Homing method 37 only defines the current position as home and is
> not meant to move a joint, but run the script yourself at the robot with the approved E-stop/STO
> within reach.

---

## Prerequisites

- Motion Master is running and reachable.
- The `MOTION_MASTER_HOSTNAME` environment variable is set (see root `README.md`).
- The devices to check are connected and discoverable via `getDevices`.
- For Stage 7, you know the SMM (Safety Motion Module) username and password for the target
  devices (`--username`/`--password`, defaulting to `Test`/`SomanetSMM`).

---

## Usage

```bash
# Check every joint in drive_configs against the live session
MOTION_MASTER_HOSTNAME=<host> npx ts-node src/humanoid_bringup/write-drive-config.ts \
  --diagnose

# Check only the joint labeled "104 ..."
MOTION_MASTER_HOSTNAME=<host> npx ts-node src/humanoid_bringup/write-drive-config.ts \
  --diagnose --device-number 104

# Commission a new robot, starting with the drive names (Roger-3: IgH positions 0, 3 and 18 are
# EtherCAT switches). Each stage asks for "yes" first.
MOTION_MASTER_HOSTNAME=<host> npx ts-node src/humanoid_bringup/write-drive-config.ts \
  --apply --assign-names --ignore_ecat_indices 0,3,18

# Drives that already carry their names
MOTION_MASTER_HOSTNAME=<host> npx ts-node src/humanoid_bringup/write-drive-config.ts \
  --apply
```

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--diagnose` | `false` | Run Stage 0 (read-only polarity check) only. Without it (and without `--apply`), the script does nothing and never connects to Motion Master. |
| `--apply` | `false` | Runs Stage 0, then Stages 1-7: assign names (1, with `--assign-names`), load the config files without `0x607C` (2), confirm, check STO then write the polarity from the config files + transition (2), set the Encoder 2 single turn offset and home every joint at the middle of its single-turn range, failing loudly on a non-zero Home Offset (3), confirm and save, report position (4), confirm and write + save software position limits (5), then confirm and apply Quick Stop / SS1 safety configuration via the SMM (7). |
| `--skip-load-configs` | `false` | Stage 2: do not load the config files onto the drives. |
| `--exclude <ids>` | *(none)* | Stage 2: comma-separated objects to leave out of every config load **in addition to `0x607C`**, e.g. `0x2001:00,0x2112` (an entry without a subindex excludes every subindex). |
| `--assign-names` | `false` | Stage 1: write the joint labels (`0x20F2:00`) from the `position_<NN>_*.csv` files by slave position. Needs `--ignore_ecat_indices` unless every slave on the bus is a drive. |
| `--ignore_ecat_indices <list>` | *(none)* | Stage 1: comma-separated IgH positions of slaves that are not drives (e.g. Beckhoff EtherCAT switches), which have no config file, e.g. `0,3,18`. |
| `--samples <n>` | `10` | Stage 4: samples of the Encoder 2 position per drive (5-10). |
| `--resolution-bits <n>` | `20` | Stage 4: Encoder 2 single-turn resolution in bits; the middle is `2^(n-1)`. |
| `--max-spread <counts>` | `16` | Stage 4: a drive whose raw Encoder 2 position varies by more than this is refused as moving. |
| `--tolerance <counts>` | `4` | Stage 4: how far from the middle the read-back adjusted position may be. |
| `--home-tolerance-deg <deg>` | `0.5` | Stage 4: largest `|Actual position|` after homing that counts as close to 0. |
| `--pause-ms <ms>` | `2000` | Stage 4: pause after printing each joint's name, before it is processed. |
| `--device-number <value>` | *(all)* | Only check the joint matching this number at `0x20F2:00`. If omitted, every config file is checked. |
| `--config-dir <path>` | `drive_configs/` (next to this script) | Directory containing the `*.config.csv` files |
| `-u, --username <value>` | `Test` | SMM login username, used by Stage 7 |
| `-p, --password <value>` | `SomanetSMM` | SMM login password, used by Stage 7 |
| `-t, --request-timeout <ms>` | `10000` | Request timeout in milliseconds for device reads |

---

## What the script does

If neither `--diagnose` nor `--apply` is passed, prints a hint and exits without connecting to
Motion Master.

### Stage 0 — diagnose

1. Scans `--config-dir` for every `*.csv` file with a `0x20F2:00` label, grouped by joint
   number (or just the one matching `--device-number`, if given). A joint number matched by
   more than one file is reported as `AMBIGUOUS CONFIG` rather than aborting the whole run.
2. Connects to Motion Master, discovers devices (`getDevices`), and reads `0x20F2:00` from
   each to group live devices by joint number the same way. A joint number matched by more
   than one live device is reported as `AMBIGUOUS DEVICE`.
3. For each joint number found in the config files:
   - If no live device shares that number: `NO LIVE DEVICE`.
   - If the config file has no `0x200D:00` entry: `NO FILE POLARITY`.
   - Otherwise reads the live device's `0x200D:00` and reports `MATCH` or `MISMATCH`.
4. Prints a table (`console.table`) with one row per joint — device number, joint label, file
   polarity, device polarity (each shown as `0 (normal)` / `1 (inverted)`), and status —
   followed by a count summary.
5. Also lists any live devices whose joint number has no matching config file at all.

The remaining stages only run with `--apply`.

### Stage 1 — assign names

Only with `--apply --assign-names`; otherwise this stage prints that it was skipped. The labels
are the `0x20F2:00` strings (`"<number> <joint name>"`, e.g. `104 head_yaw`) that Stage 0 and
the later stages match on, so a robot whose drives were never labelled can be named in the same
run.

> **Applying this step assumes EtherCAT network is wired exactly same between the robots.**
> **Assuming monotonic cabling and position 0, 3, 18 as EtherCAT switch positions.**
> Both are printed as a large warning before the table and again before the confirmation (the
> second line lists whatever you pass to `--ignore_ecat_indices`).

Each file in `drive_configs/` is named `position_<NN>_<name>_config.csv`; `NN` is the position it
was exported from, and the file's own `0x20F2:00` row holds the label. The label is written to
the drive at the same position on the connected robot.

**The files and Motion Master count differently.** The files are always generated with the IgH
master, which counts from 0 and includes every slave on the bus, such as the Beckhoff EtherCAT
switches. On Roger-3 positions 0, 3 and 18 are switches, so they have no file and the file
numbers have gaps. OBLAC's Motion Master uses SOEM, which only reports the drives. Pass the IgH
positions of the slaves that are not drives with `--ignore_ecat_indices`; the file position is
converted to a drive number by taking out the ignored positions that come before it. With
`--ignore_ecat_indices 0,3,18`:

| File | Drive on the network |
|---|---|
| `position_01_*` | 1st (position 0 is before it) |
| `position_02_*` | 2nd |
| `position_04_*` | 3rd (positions 0 and 3 are before it) |
| `position_19_*` | 17th (positions 0, 3 and 18 are before it) |
| `position_33_*` | 31st, the last |

"Nth drive" is the Nth device in bus order, so SOEM's 1-based and IgH's 0-based numbering make no
difference. Without the flag nothing is ignored and position 0 is the 1st drive. The step is only
correct when the cabling is monotonic, i.e. the drives are wired in the same order as on the robot
the files came from.

It prints the pairing table (file position, drive number, current label, new label), warns when
the number of drives differs from the number of config files, and asks for `yes`. Each label is
written, read back, and the drive's parameters saved. A file at an ignored position, a file with
no drive at its position, two files claiming one position or two files with the same joint number
is reported, and any failure stops the run. Afterwards Stage 0 is repeated so the later stages
see the new names.

### Stage 2 — load the config files, without the Home Offset

Replaces running `load-drive-configs.ts` separately: each config file that Stage 0 paired with a
live drive is loaded onto that drive. Skipped with `--skip-load-configs`.

> **The Home Offset (`0x607C`) is never loaded.** The console prints
> `[EXCLUDE] 0x607C (every subindex): Home Offset must stay 0 (no home offset is used)` before the
> table and, per joint, the value the file holds, e.g.
> `[EXCLUDE] 104 head_yaw: 0x607C:00 = 1048576 not loaded (excluded 0x607C)`. No home offset is
> used: Stage 4 homes every joint where it is and the Home Offset must stay `0`. Some exported
> configs carry one (Roger-3: `1048576`, i.e. 360°, in 5 of the files).

- `--exclude <ids>` adds more objects to leave out, e.g. `--exclude 0x2001:00,0x2112` to keep each
  drive's own commutation offset and Encoder 2 configuration (an entry without a subindex excludes
  every subindex). Nothing else is excluded by default, so a config exported from another robot
  overwrites that robot's calibration values unless you list them.
- The strategy is `merge`: Motion Master reads the drive's existing `config.csv`, applies the
  file's rows on top, and writes the result back, so an excluded object keeps the value the drive
  already has.
- Always left out: the SMM safety parameters (the safety layer ignores plain writes to them; Stage
  7 applies them through the authenticated SMM flow) and the joint label `0x20F2:00`, so a label
  can never end up on the wrong drive.
- Prints a table (joint, file, parameters to send, parameters excluded) and asks for `yes`. A
  failure stops the run. Afterwards Stage 0 is repeated so the later stages see what the drives
  now hold.

### Stage 3 — apply the polarity from the config files

Every `MATCH` or `MISMATCH` row is eligible; if there are none, prints "nothing to apply" and
stops. `NO LIVE DEVICE` / `NO FILE POLARITY` / `AMBIGUOUS CONFIG` / `AMBIGUOUS DEVICE` rows are
never eligible. Otherwise prints the list of eligible joints (current vs. new value, and whether
each will get `WRITE + TRANSITION` or `TRANSITION ONLY`) and prompts
`Type "yes" to proceed on these N device(s), anything else to abort:`. Anything other than
exactly `yes` aborts with no changes made.

1. **Checks STO** (Safe Torque Off — bit 0 of `0x6621:01`) on every eligible device. If STO is
   active, or its status can't be read, on *any* device, throws immediately — naming every
   affected device — which aborts the whole `--apply` run with `[FATAL] ...` before anything is
   written. This is because the controlword transition below cannot succeed while STO is active.
2. For each `MISMATCH` device: writes `0x200D:00` (the polarity from the config file) via a plain
   SDO download and reads it back to verify. For each `MATCH` device, nothing is written. Then,
   for every eligible device (provided the `MISMATCH` write/verify above succeeded), cycles the
   controlword (`0x6040:00`) through `0 -> 6 -> 7` — clear, Shutdown, Switch On, 200 ms apart.
   Prints a per-device summary (`APPLIED` / `TRANSITIONED` / `VERIFY FAILED` / `WRITE FAILED` /
   `POLARITY WRITTEN, CONTROLWORD TRANSITION FAILED` / `CONTROLWORD TRANSITION FAILED`). The
   polarity is saved by Stage 4.

### Stage 4 — set the Encoder 2 single turn offset and home

Start with the humanoid **mechanically zeroed (or at the zero you want)**: every joint is homed
where it is, so that position becomes zero. The Home Offset is not used to move the zero — no
correction is added to it.

The goal is to set Encoder 2 single turn offset (`0x2112:06`) so that Encoder 2 feedback
Adjusted position (`0x2113:02`) is exactly 2^19, the middle of a 2^20 range, giving the joint
the maximum range of motion without the single-turn value overflowing.

**Planning (read-only), for every joint that completed Stage 3:**

1. Reads `--samples` (5-10, default 10) samples of the raw (`0x2113:01`) and adjusted
   (`0x2113:02`) position. A drive whose raw value varies by more than `--max-spread` counts
   (default 16) is refused as moving, and so is one that is operation enabled or in fault.
2. Calculates the offset from the raw position:

   ```
   IF raw < resolution / 2    offset = raw + 2^19
   ELSE                       offset = raw - 2^19
   ```

3. Prints a table: adjusted position before, raw position, offset now, calculated offset, the
   adjusted position expected afterwards (`2^19`), the Home Offset now, and a status.
   A joint that cannot be processed stops the stage before anything is written — homing only
   some of the joints would leave the robot half-commissioned.
4. Prints a large warning, because the drives are energized, and asks for `yes`.

**Per joint, one after the other** — its assigned name is printed first, followed by a pause of
`--pause-ms` (default 2000 ms) so you can see which joint it is:

1. If the Home Offset (`0x607C:00`) is not `0` (e.g. a loaded config carries one), it is reset
   to `0` and read back. This is reported as a warning.
2. Writes `0x2112:06` and sets *Restore home position when loading configuration* (`0x2005:02`)
   to `1` (YES), saves all parameters (`0x1010`), then reads `0x2113:02` back. It must be `2^19`
   within `--tolerance` counts (default 4); on anything else the previous offset and restore flag
   are written back and saved, and the run stops.
3. **Homes at the current position with method 37**: Modes of operation (`0x6060:00`) = 6,
   Homing method (`0x6098:00`) = 37, controlword `0x06 -> 0x07 -> 0x0F`, then `0x1F` to start
   homing, and waits for Homing attained (statusword bit 12). Method 37 only defines the current
   position as home and is not meant to move the joint, but the drive is energized. Afterwards the
   drive is returned to Switched on disabled and its previous mode of operation is restored, also
   when the homing failed.
4. **Checks that the Home Offset (`0x607C:00`) is still `0`**. If not, the run **fails loudly**:
   a large warning naming the joint and the value, and the run stops. It also checks that the
   Actual position (`0x6064:00`) is within `--home-tolerance-deg` (default 0.5°) of 0.

The stage stops at the first joint that fails, prints the summary table, and throws, so Stages 5,
5 and 7 do not run.

The calculation assumes the adjusted position is the raw position minus the offset. The script
warns when a drive does not behave that way, and the read-back after writing catches it.
`--resolution-bits` (default 20) sets the single-turn resolution if your encoder differs.

### Stage 5 — save and report actual position

Asks for `yes`, then makes sure *Restore home position when loading configuration* (`0x2005:02`)
reads `1` on every homed device (writing it if not — the homed position only persists with it),
saves all parameters (`0x1010`: Store parameters), and prints each device's Actual
position (`0x6064:00`, in degrees, assuming a 20-bit position encoder) and Home Offset
(`0x607C:00`). A Home Offset that is not `0` fails loudly. Answering anything but `yes` skips the
save and only reports.

### Stage 6 — apply and save software position limits

For every device that went through Stage 3 (`MATCH` and `MISMATCH` alike) whose config file has
both `0x607D:01` (min) and `0x607D:02` (max):

1. Converts both values to degrees, **assuming a 20-bit position encoder** (1,048,576
   counts/revolution) — logged as `[INFO]`, same assumption as Stages 4-5.
2. Prints the list of devices that would be changed and, if any Stage 3 device's config file is
   missing one or both limits, lists which ones will be skipped.
3. Prompts
   `Type "yes" to write these N software position limit(s) and save all parameters, anything else to abort:`.
   Anything other than exactly `yes` aborts with nothing written or saved.
4. On confirmation, for each device: writes `0x607D:01` and `0x607D:02` via plain SDO downloads
   and reads both back to verify. **Only once verified**, calls `saveConfig` (`0x1010`: Store
   parameters) to persist that device's entire current parameter set. Prints a per-device
   summary (device, joint, both limits in degrees, and `APPLIED & SAVED` /
   `APPLIED, SAVE FAILED (...)` / `VERIFY FAILED` / `WRITE FAILED`).

### Stage 7 — Quick and SS1 Configuration

Applies the same Quick Stop /
SS1 (Safe Stop 1) safety configuration as
[`Damping Configuration/configure-damping-parameters.promise.ts`](../Damping%20Configuration/configure-damping-parameters.promise.ts),
to every device that went through Stage 3 (`MATCH` and `MISMATCH` alike):

1. For each device, reads the current values of:
   - **Quick stop option code** (`0x605A:00`) and **ASC duration** (`0x2010:12`) — plain CoE
     objects, read via a direct SDO upload.
   - Three **SMM safety parameters** (`0x2641:01`, `0x2650:01`, `0x6651:01`) — read via the SMM
     object dictionary (`getSmmOdParameters`). A device with no serial number can't be addressed
     for this and is reported `NO SERIAL NUMBER`.
2. Prints a preview table — device, joint, quick stop and ASC duration (`current -> target`),
   all three SS1 safety values (`current->target`), and `PENDING` / `ALREADY SET`.
3. If every checked device already matches the target, prints "nothing to apply" and stops.
   Otherwise prompts
   `Type "yes" to apply Quick Stop / SS1 configuration to these N device(s), anything else to abort:`.
   Anything other than exactly `yes` aborts with nothing written.
4. On confirmation, for each device that needs a change:
   - If the quick stop / ASC duration values differ: writes both via a direct SDO download, runs
     a **read-only** sanity check on the dual-encoder / torque-sensor prerequisites (`0x2112:01`
     expected `2`, `0x2150:01` expected `1` — logged as `[WARN]` if not, but never blocks), then
     calls `saveConfig` (`0x1010`: Store parameters) to persist them.
   - If any of the three SS1 safety values differ: rebuilds the device's full SMM configuration
     CSV from its current live parameters, patches in the three target values, and applies it
     through the authenticated SMM configuration flow (login → transmit → verify → validate),
     using `--username`/`--password` — every other SMM parameter round-trips unchanged. Then
     triggers an SMM restart and waits 2 seconds for it to settle.
   - Re-reads all five values to verify, and prints a per-device summary
     (`APPLIED & SAVED` / `VERIFY FAILED` / `NON-SAFETY WRITE FAILED (...)` /
     `SAFETY WRITE FAILED (...)`, each with a `(sanity check warning)` suffix if that check
     didn't pass).

Quick stop / ASC duration are saved directly (`0x1010`); the SS1 safety parameters are
persisted by the SMM validation flow itself — both are independent of whatever Stages 5/5 did.

---

## Example output

```
=== Stage 0: Diagnose system polarity for all devices ===

Discovered 31 device(s) on the network.

┌─────────┬────────┬───────────────────┬────────────────┬───────────────────┬────────────┐
│ (index) │ Device │ Joint             │ File Polarity   │ Device Polarity   │ Status     │
├─────────┼────────┼───────────────────┼────────────────┼───────────────────┼────────────┤
│ 0       │ '101'  │ '101 torso_yaw'   │ '0 (normal)'    │ '0 (normal)'      │ 'MATCH'    │
│ 1       │ '102'  │ '102 torso_right' │ '1 (inverted)'  │ '1 (inverted)'    │ 'MATCH'    │
│ 2       │ '103'  │ '103 torso_left'  │ '0 (normal)'    │ '0 (normal)'      │ 'MATCH'    │
│ 3       │ '104'  │ '104 head_yaw'    │ '0 (normal)'    │ '1 (inverted)'    │ 'MISMATCH' │
└─────────┴────────┴───────────────────┴────────────────┴───────────────────┴────────────┘
=== Summary ===
  Joints checked:       31
  Match:                30
  Mismatch:             1
  No live device:       0
  No file polarity:     0
  Ambiguous config:     0
  Ambiguous device:     0
================
```

With `--apply`, the run continues with Stages 1-5. Their output has not been captured from a robot
yet, so it is not reproduced here; the tables are described in the stage sections above. Stages 6
and 7 look like this:

```
=== Stage 6: Apply software position limits ===

[INFO] Assuming a 20-bit position encoder (1048576 counts/revolution) to convert 0x607D:01/02 to degrees.

The following 4 device(s) will have their software position limits (0x607D:01/02) written from their config file:

┌─────────┬────────┬────────────────────┬────────────────┬────────────────┐
│ (index) │ Device │ Joint              │ Min Limit (deg)│ Max Limit (deg)│
├─────────┼────────┼────────────────────┼────────────────┼────────────────┤
│ 0       │ '101'  │ '101 torso_yaw'    │ '-45.00'       │ '45.00'        │
│ 1       │ '102'  │ '102 torso_right'  │ '-30.00'       │ '30.00'        │
│ 2       │ '103'  │ '103 torso_left'   │ '-30.00'       │ '30.00'        │
│ 3       │ '104'  │ '104 head_yaw'     │ '-51.00'       │ '51.00'        │
└─────────┴────────┴────────────────────┴────────────────┴────────────────┘

Once written and verified, ALL parameters currently on each device will be SAVED
(0x1010: Store parameters), so they persist across a power cycle.

Type "yes" to write these 4 software position limit(s) and save all parameters, anything else to abort: yes

=== Software position limit summary ===
┌─────────┬────────┬────────────────────┬────────────────┬────────────────┬───────────────────┐
│ (index) │ Device │ Joint              │ Min Limit (deg)│ Max Limit (deg)│ Status            │
├─────────┼────────┼────────────────────┼────────────────┼────────────────┼───────────────────┤
│ 0       │ '101'  │ '101 torso_yaw'    │ '-45.00'       │ '45.00'        │ 'APPLIED & SAVED' │
│ 1       │ '102'  │ '102 torso_right'  │ '-30.00'       │ '30.00'        │ 'APPLIED & SAVED' │
│ 2       │ '103'  │ '103 torso_left'   │ '-30.00'       │ '30.00'        │ 'APPLIED & SAVED' │
│ 3       │ '104'  │ '104 head_yaw'     │ '-51.00'       │ '51.00'        │ 'APPLIED & SAVED' │
└─────────┴────────┴────────────────────┴────────────────┴────────────────┴───────────────────┘
Parameters were saved (0x1010) on every successfully applied device — they now persist across a power cycle.

=== Stage 7: Quick and SS1 Configuration ===

┌─────────┬────────┬────────────────────┬─────────────────────────┬──────────────────────────┬────────────────────────────────┬───────────┐
│ (index) │ Device │ Joint              │ Quick Stop (0x605A:00)  │ ASC Duration (0x2010:12)│ SS1 Safety Params              │ Status    │
├─────────┼────────┼────────────────────┼─────────────────────────┼──────────────────────────┼────────────────────────────────┼───────────┤
│ 0       │ '101'  │ '101 torso_yaw'    │ '0 -> 9'                │ '0 -> 2500'              │ '-->0, -->1, -->3000'          │ 'PENDING' │
│ 1       │ '102'  │ '102 torso_right'  │ '9 -> 9'                │ '2500 -> 2500'           │ '0->0, 1->1, 3000->3000'       │ 'ALREADY SET' │
│ 2       │ '103'  │ '103 torso_left'   │ '0 -> 9'                │ '0 -> 2500'              │ '-->0, -->1, -->3000'          │ 'PENDING' │
│ 3       │ '104'  │ '104 head_yaw'     │ '9 -> 9'                │ '2500 -> 2500'           │ '0->0, 1->1, 3000->3000'       │ 'ALREADY SET' │
└─────────┴────────┴────────────────────┴─────────────────────────┴──────────────────────────┴────────────────────────────────┴───────────┘

2 device(s) need changes. Quick stop / ASC duration are written via a plain SDO
download then saved (0x1010: Store parameters). The three SS1 safety parameters are written via the
authenticated SMM configuration flow (login/transmit/verify/validate/restart) using --username/--password
(currently "Test"), preserving every other SMM parameter already on the device.

Type "yes" to apply Quick Stop / SS1 configuration to these 2 device(s), anything else to abort: yes

  [101] Sanity check — dual encoder & torque sensor configuration:
  [103] Sanity check — dual encoder & torque sensor configuration:

=== Quick Stop / SS1 configuration summary ===
┌─────────┬────────┬────────────────────┬────────────────────┐
│ (index) │ Device │ Joint              │ Status             │
├─────────┼────────┼────────────────────┼────────────────────┤
│ 0       │ '101'  │ '101 torso_yaw'    │ 'APPLIED & SAVED'  │
│ 1       │ '103'  │ '103 torso_left'   │ 'APPLIED & SAVED'  │
└─────────┴────────┴────────────────────┴────────────────────┘
Quick stop / ASC duration were saved (0x1010). SS1 safety parameters were validated and persisted via the
SMM configuration flow — both are now persistent on every successfully applied device.
```

Stages 6 and 7 cover every device that went through Stage 3. Devices 101/103 are saved by Stage 7
only when their quick-stop/SS1 values were not already at target.

### If STO is active

If any device's Safe Torque Off is active when Stage 3 runs, the whole `--apply` run stops
immediately, before anything in Stage 3 is written:

```
=== Stage 3: Apply system polarity from the config files ===

[FATAL] STO (Safe Torque Off) is active on: 104 (104 head_yaw). The controlword transition 0 -> 6 -> 7 cannot succeed while STO is active. Release STO on the affected device(s) and re-run --apply.
```

Release STO on the named device(s) and re-run `--apply` — the stages will need to be confirmed
again, since nothing from that run was written.

---

## Using this as a template

Nothing here is specific to one robot, but three things are specific to *a* robot and need
checking before the first `--apply` run.

### 1. Add your joint configs

Export a `config.csv` per drive and put them in `drive_configs/`. Any filename ending in `.csv`
is picked up; the directory ships empty apart from a `.gitkeep` note, which the scripts ignore.

A naming convention of `<number>_<joint_name>.config.csv` (e.g. `104_head_yaw.config.csv`) is
worth following: it makes the directory readable. To have Stage 1 name the drives, name the files
`position_<NN>_<name>_config.csv` instead (see Stage 1).

### 2. Label every drive and every config file

Matching is done on the `0x20F2:00` label, in the form `"<number> <joint name>"`. Run
`write-drive-config.ts --apply --assign-names` ([Stage 1](#stage-1--assign-names)) to write the
labels to a robot's drives in one command, taken from the config files, which must then be named
`position_<NN>_<name>_config.csv` after the IgH position they were exported from:

```
0x20f2,  0,  104 head_yaw
```

The **leading number is the key** — the joint name is for humans. Both the live drive and its
config file must carry the same number. Give each joint a distinct one; a duplicate is reported
as `AMBIGUOUS CONFIG` or `AMBIGUOUS DEVICE` and skipped rather than guessed at.

Pick any numbering that suits the machine. A grouped scheme (1xx torso and head, 2xx/3xx left
and right arms, 4xx/5xx left and right legs) keeps related joints together and makes a partial
bring-up easy to scope with `--device-number`.

A drive with no `0x20F2:00` object, or an unlabelled one, is skipped — so a non-CiA402 device on
the same network is harmless.

### 3. Check the machine-specific assumptions

| What | Where | Why it matters |
|---|---|---|
| **Encoder resolution** | `ASSUMED_ENCODER_RESOLUTION_BITS` in `write-drive-config.ts` (20 bits) | Stages 4-6 convert the Actual position and the software position limits between counts and degrees. A drive with a different resolution gets wrong degrees, so set this to your encoders or make it per-joint. (The Encoder 2 single-turn resolution used in Stage 4 is `--resolution-bits`.) |
| **Home Offset** | Stage 4 | No correction is added to the Home Offset (`0x607C:00`): it must be `0` after homing, otherwise the run fails loudly. Note that some exported configs carry one (Roger-3: `1048576`, i.e. 360°, in 5 of the files) — Stage 2 does not load `0x607C` (and says so), and Stage 4 still resets a non-zero value to `0` before homing. |
| **Quick stop / SS1 targets** | The Stage 7 constants | These are commissioned values, not defaults. Confirm them against your own safety concept before applying. |
| **SMM credentials** | `--username` / `--password` (default `Test` / `SomanetSMM`) | Stage 7 needs the real credentials for your drives. |

### 4. Bring up in this order

1. Mechanically zero the humanoid (or put every joint at the zero you want).
2. `write-drive-config.ts --diagnose` — read-only polarity check.
3. `write-drive-config.ts --apply [--assign-names --ignore_ecat_indices 0,3,18] [--exclude ...]` —
   staged, each stage confirmed: names (1), config load without `0x607C` (2), polarity (3),
   Encoder 2 single turn offset and homing (4), save (5), limits (6), safety (7).
---

# humanoid_bringup — drive bring-up template

A starting point for bringing up the drives of a multi-joint humanoid: drop your exported
joint configs into `drive_configs/`, and the two scripts here pair each live drive with its
config file and act on it.

| Script | What it does |
|---|---|
| `assign-names.ts` | **Step 1.** Writes each drive's joint label (`0x20F2:00`) from the config files, by EtherCAT slave position, in a single command. Run it first on a robot whose drives are not labelled yet. |
| `write-drive-config.ts` | Diagnoses **system polarity** (`0x200D:00`) against each config file, and can apply polarity, home offset, software position limits and quick stop / SS1 safety configuration in confirmed stages. |
| `load-drive-configs.ts` | Loads each joint's config file onto its drive, with `--exclude` for objects that must not be overwritten (per-drive calibration values). |

**This directory ships with `drive_configs/` empty.** Nothing runs until you put config files
in it — see [Using this as a template](#using-this-as-a-template) below.

Most of this document describes `write-drive-config.ts`, the larger of the two.

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
| **1** | `--apply` only | Ask for confirmation before applying any polarity change. |
| **2** | `--apply` only | Check STO (Safe Torque Off) is inactive on every device — abort with a warning if not. Then write the polarity change (`MISMATCH` devices only) and cycle the controlword `0 -> 6 -> 7` on every `MATCH`/`MISMATCH` device. Not saved by itself — see the note below. |
| **3** | `--apply` only | **Only** for devices whose polarity did *not* match their config file (`MISMATCH`) that completed Stage 2 — `MATCH` devices are left alone entirely. Inverting polarity at a non-zero angle adds a full revolution to the Actual position value, in either direction — always fixed by `+360°`. To make that fix exact, **checks and writes the home offset (`0x607C:00`) to `0`** if it isn't already (the only write this stage makes), reads the Actual position value (`0x6064:00`, for the table only), and *calculates* the pending home offset as `+360°` worth of counts for every such device. |
| **4** | `--apply` only | Asks for confirmation again, then — only if restore home position (`0x2005:02`) reads `1` on every device from Stage 2 — writes the home offset adjustments Stage 3 calculated and, once verified, **saves all parameters** on that device (`0x1010`: Store parameters) — see the note below. Finally reads and prints the Actual position value (`0x6064:00`), in degrees, for **every** device that went through Stage 2 (`MATCH` and `MISMATCH` alike), regardless of the above. |
| **5** | `--apply` only | Asks for confirmation, then writes the software position limits (`0x607D:01` min, `0x607D:02` max) from each device's matched config file to that device, for every Stage 2 device whose config file has both values, and — once verified — **saves all parameters** on that device. Prints a summary in degrees. |
| **7** | `--apply` only | *(there is no Stage 6)* "Quick and SS1 Configuration" — the same parameters `Damping Configuration/configure-damping-parameters.promise.ts` writes. Quick stop option code (`0x605A:00`) and ASC duration (`0x2010:12`) are written via plain SDO and saved; three SMM safety parameters (`0x2641:01`, `0x2650:01`, `0x6651:01`) are written via the authenticated SMM configuration flow using `--username`/`--password`. Only devices whose values differ from target are touched. Asks for confirmation first. |

Stage 2's controlword transition (`0x6040:00`: clear -> Shutdown -> Switch On) makes a
`MISMATCH` device's newly written polarity take effect; `MATCH` devices have nothing written,
but the same transition is still applied.

> **On STO:** before cycling the controlword, Stage 2 reads STO (Safe Torque Off — bit 0 of the
> Safety Statusword's first byte, `0x6621:01`) on every device about to be transitioned. The
> controlword transition `0 -> 6 -> 7` cannot succeed while STO is active (the drive can't leave
> "Switch on disabled"). If STO is active — or unreadable — on *any* device, the entire `--apply`
> run aborts immediately with `[FATAL] STO (Safe Torque Off) is active on: ...`, naming the
> affected device(s), before anything is written. Release STO on those devices and re-run
> `--apply`.

> **On persistence:** Stage 2's polarity write and Stage 3/4's home offset writes are plain SDO
> downloads — none is saved by itself. A device's parameters become persistent if *either*:
> (a) Stage 4 reaches its save step for that device, which requires the device to have gone
> through Stage 3's polarity-inversion correction, restore home position (`0x2005:02`) reading
> `1` on every device from Stage 2, and your confirmation at Stage 4's prompt; or (b) Stage 5 saves it,
> which requires its config file to have both software position limits and your confirmation at
> Stage 5's prompt. Either save persists that device's *entire* current parameter set at that
> moment, including its Stage 2 polarity write — not just the object that stage was about. A
> device that never reaches either save step keeps every write from this script live-only, and
> loses it on the next power cycle or config reload. Stage 7 is independent of this: it saves
> its own quick-stop objects directly (`0x1010`) and persists its own SMM safety objects via the
> SMM validation flow, regardless of what Stages 4/5 did.

---

## Background

`drive_configs/` holds one exported `config.csv` per joint. Motion Master names an export
after the drive's product ID and serial number (e.g. `6401-01-0000338-2613.config.csv`), which
says nothing about which physical joint the file belongs to, so matching cannot rely on the
filename alone.

Every config file — and every live device — carries a human-readable label at `0x20F2:00` in
the form `"<joint number> <joint name>"`, e.g.:

```
0x20f2,  0,  104 head_yaw
```

This script uses that label, not the filename, to pair a config file with a live device. When
`--device-number` (e.g. `104`) is given, only the joint whose label starts with that number is
checked; otherwise every config file with a `0x20F2:00` label is checked.

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

# Diagnose, then (after typing "yes" to confirm) write 0x200D:00 on every mismatching device
# and cycle the controlword 0 -> 6 -> 7 on every matching or mismatching device — after
# confirming STO is inactive on every device.
MOTION_MASTER_HOSTNAME=<host> npx ts-node src/humanoid_bringup/write-drive-config.ts \
  --apply
```

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--diagnose` | `false` | Run Stage 0 (read-only polarity check) only. Without it (and without `--apply`), the script does nothing and never connects to Motion Master. |
| `--apply` | `false` | Runs Stage 0, then Stages 1-5 and 7: confirm, check STO then write polarity + transition, reset home offset to zero and calculate the `+360°` polarity-inversion correction, confirm again to check restore home position, write the offsets and save all parameters on the devices touched, report position, confirm and write + save software position limits, then confirm and apply Quick Stop / SS1 safety configuration via the SMM. |
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

### Stage 1 — confirm

Every `MATCH` or `MISMATCH` row is eligible; if there are none, prints "nothing to apply" and
stops (Stages 2-5 and 7 don't run). `NO LIVE DEVICE` / `NO FILE POLARITY` / `AMBIGUOUS CONFIG` /
`AMBIGUOUS DEVICE` rows are never eligible. Otherwise prints the list of eligible joints
(current vs. new value, and whether each will get `WRITE + TRANSITION` or `TRANSITION ONLY`)
and prompts `Type "yes" to proceed on these N device(s), anything else to abort:`. Anything
other than exactly `yes` aborts with no changes made (Stages 2-5 and 7 don't run).

### Stage 2 — check STO, apply polarity, controlword transition

1. **Checks STO** (Safe Torque Off — bit 0 of `0x6621:01`) on every eligible device. If STO is
   active, or its status can't be read, on *any* device, throws immediately — naming every
   affected device — which aborts the whole `--apply` run with `[FATAL] ...` before anything is
   written. This is because the controlword transition below cannot succeed while STO is active.
2. For each `MISMATCH` device: writes `0x200D:00` via a plain SDO download and reads it back to
   verify. For each `MATCH` device, nothing is written. Then, for every eligible device (provided
   the `MISMATCH` write/verify above succeeded), cycles the controlword (`0x6040:00`) through
   `0 -> 6 -> 7` — clear, Shutdown, Switch On, 200 ms apart. Prints a per-device summary
   (`APPLIED` / `TRANSITIONED` / `VERIFY FAILED` / `WRITE FAILED` /
   `POLARITY WRITTEN, CONTROLWORD TRANSITION FAILED` / `CONTROLWORD TRANSITION FAILED`). No
   save/persist command is issued in this stage — the polarity value is live-only for now, and
   reverts on the next power cycle or config reload *unless* Stage 4 or Stage 5 later saves that
   device (see the persistence note above).

### Stage 3 — reset home offset and calculate polarity-inversion correction

**Only** for devices with status `MISMATCH` — `MATCH` devices had nothing written to them in
Stage 2 and are left alone entirely here, not even listed. Of the `MISMATCH` devices, only
those whose Stage 2 status was `APPLIED` are processed; the rest are reported as `SKIPPED`.

Inverting system polarity at a non-zero angle adds a full revolution to the Actual position
value, regardless of direction: at `+90°`, inverting moves it to `-(360+90) = -450°` (not
`-90°`); at `-90°`, it moves to `-(360-90) = -270°` (not `+90°`). In both cases the correction
is the same fixed `+360°` worth of counts — it does **not** depend on the position's sign or
magnitude. For that correction to be exact regardless of whatever was already on the device:

1. **Checks the device's current home offset (`0x607C:00`)** and, if it isn't already `0`,
   **writes it to `0`** and verifies the readback — the only write this stage makes. If that
   verification fails, the device is reported `RESET TO ZERO FAILED` and skipped.
2. Reads the Actual position value (`0x6064:00`) and converts it to degrees, **assuming a
   20-bit position encoder** (1,048,576 counts/revolution, logged as `[INFO]`) — purely for the
   printed table; it no longer affects the calculation.
3. Calculates the pending home offset as `+360°` worth of counts (i.e. `0 + 360°`, since the
   offset was just reset) for **every** device that reaches this point.
4. Prints a summary table — device number, joint label, home offset before the reset (in
   degrees), position in degrees, and status (`RESET TO 0, +360° PENDING IN STAGE 4` /
   `RESET TO ZERO FAILED (...)` / `SKIPPED (stage 2 did not complete)` / `FAILED (...)`).

### Stage 4 — apply home offsets, save, and report actual position

1. If Stage 3 calculated no corrections, prints "no home offset corrections to apply" and
   skips straight to the position report (step 5 below).
2. Otherwise prints the list of devices that would be changed (position, and the calculated new
   home offset in counts) and prompts
   `Type "yes" to apply these N home offset adjustment(s) and save all parameters, anything else to abort:`.
   Anything other than exactly `yes` aborts — no home offset is written and nothing is saved
   (the position report in step 5 still runs).
3. On confirmation, **checks restore home position (`0x2005:02`) on every device that went
   through Stage 2** (`MATCH` and `MISMATCH` alike) and prints the result. If any device reads
   anything other than `1` (including a read failure), the whole save step is refused — no
   home offset is written and nothing is saved (the position report in step 5 still runs).
4. Otherwise, for each flagged device: writes the calculated home offset (`0x607C:00`) via a
   plain SDO download and reads it back to verify. **Only once verified**, calls `saveConfig`
   (`0x1010`: Store parameters) to persist that device's entire current parameter set — which
   also makes its Stage 2 polarity write persistent. Prints a per-device summary
   (`APPLIED & SAVED` / `APPLIED, SAVE FAILED (...)` / `VERIFY FAILED` / `WRITE FAILED`).
5. Reads the Actual position value (`0x6064:00`) again and prints it in degrees for **every**
   device that went through Stage 2 — `MATCH` and `MISMATCH` alike, regardless of what happened
   above — so you can see where every joint stands after the whole process.

### Stage 5 — apply and save software position limits

For every device that went through Stage 2 (`MATCH` and `MISMATCH` alike) whose config file has
both `0x607D:01` (min) and `0x607D:02` (max):

1. Converts both values to degrees, **assuming a 20-bit position encoder** (1,048,576
   counts/revolution) — logged as `[INFO]`, same assumption as Stages 3-4.
2. Prints the list of devices that would be changed and, if any Stage 2 device's config file is
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

*(There is no Stage 6 — this stage kept its original number.)* Applies the same Quick Stop /
SS1 (Safe Stop 1) safety configuration as
[`Damping Configuration/configure-damping-parameters.promise.ts`](../Damping%20Configuration/configure-damping-parameters.promise.ts),
to every device that went through Stage 2 (`MATCH` and `MISMATCH` alike):

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
persisted by the SMM validation flow itself — both are independent of whatever Stages 4/5 did.

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

With `--apply`, the run above continues (after the summary) with every `MATCH`/`MISMATCH`
joint — here 3 `MATCH` and 1 `MISMATCH`:

```
=== Stage 1: Confirm applying polarity changes ===

The following 4 joint(s) will have their controlword cycled 0 -> 6 -> 7, and 1 of them will
first have 0x200D:00 written to match their config file:

┌─────────┬────────┬────────────────────┬───────────────────┬──────────────────┬─────────────────────┐
│ (index) │ Device │ Joint              │ Current (device)   │ New (from file)  │ Action              │
├─────────┼────────┼────────────────────┼───────────────────┼──────────────────┼─────────────────────┤
│ 0       │ '101'  │ '101 torso_yaw'    │ '0 (normal)'       │ '0 (normal)'     │ 'TRANSITION ONLY'   │
│ 1       │ '102'  │ '102 torso_right'  │ '1 (inverted)'     │ '1 (inverted)'   │ 'TRANSITION ONLY'   │
│ 2       │ '103'  │ '103 torso_left'   │ '0 (normal)'       │ '0 (normal)'     │ 'TRANSITION ONLY'   │
│ 3       │ '104'  │ '104 head_yaw'     │ '1 (inverted)'     │ '0 (normal)'     │ 'WRITE + TRANSITION'│
└─────────┴────────┴────────────────────┴───────────────────┴──────────────────┴─────────────────────┘

Mismatching devices get 0x200D:00 written directly to their live parameter set only — NOT saved/persisted.
Every listed device then has its controlword (0x6040:00) cycled 0 -> 6 -> 7.

Type "yes" to proceed on these 4 device(s), anything else to abort: yes

=== Stage 2: Apply polarity changes and controlword transition ===

=== Apply summary ===
┌─────────┬────────┬────────────────────┬────────────────┐
│ (index) │ Device │ Joint              │ Status         │
├─────────┼────────┼────────────────────┼────────────────┤
│ 0       │ '101'  │ '101 torso_yaw'    │ 'TRANSITIONED' │
│ 1       │ '102'  │ '102 torso_right'  │ 'TRANSITIONED' │
│ 2       │ '103'  │ '103 torso_left'   │ 'TRANSITIONED' │
│ 3       │ '104'  │ '104 head_yaw'     │ 'APPLIED'      │
└─────────┴────────┴────────────────────┴────────────────┘
Reminder: polarity writes were NOT saved — they apply only until the next power cycle or config reload.

=== Stage 3: Reset home offset and calculate polarity-inversion correction ===

[INFO] Inverting system polarity at a non-zero angle adds a full revolution to the Actual
position value, in either direction. Every inverted device's home offset (0x607C:00) is
first reset to 0 (if not already), then corrected by +360° worth of counts (assuming a
20-bit position encoder, 1048576 counts/revolution).

┌─────────┬────────┬────────────────┬─────────────────────────┬────────────────┬───────────────────────────────────────┐
│ (index) │ Device │ Joint          │ Home Offset Before (deg)│ Position (deg) │ Status                                 │
├─────────┼────────┼────────────────┼─────────────────────────┼────────────────┼───────────────────────────────────────┤
│ 0       │ '104'  │ '104 head_yaw' │ '0.00'                  │ '-450.00'      │ 'RESET TO 0, +360° PENDING IN STAGE 4' │
└─────────┴────────┴────────────────┴─────────────────────────┴────────────────┴───────────────────────────────────────┘

=== Stage 4: Apply home offsets and report actual position ===

The following 1 device(s) will have their home offset (0x607C:00) adjusted:

┌─────────┬────────┬────────────────┬─────────────────┬───────────────────────────┐
│ (index) │ Device │ Joint          │ Position (deg)  │ New Home Offset (counts)  │
├─────────┼────────┼────────────────┼─────────────────┼───────────────────────────┤
│ 0       │ '104'  │ '104 head_yaw' │ '-450.00'        │ 1048576                   │
└─────────┴────────┴────────────────┴─────────────────┴───────────────────────────┘

Once written and verified, ALL parameters currently on each device — including this home
offset and any polarity change from Stage 2 — will be SAVED (0x1010: Store parameters), so
they persist across a power cycle.

Type "yes" to apply these 1 home offset adjustment(s) and save all parameters, anything else to abort: yes

Checking restore home position (0x2005:02) on all 4 drive(s) before saving...

┌─────────┬────────┬────────────────────┬────────────────────────────────┬────────┐
│ (index) │ Device │ Joint              │ Restore Home Position (0x2005:02) │ Status │
├─────────┼────────┼────────────────────┼────────────────────────────────┼────────┤
│ 0       │ '101'  │ '101 torso_yaw'    │ 1                                │ 'OK'   │
│ 1       │ '102'  │ '102 torso_right'  │ 1                                │ 'OK'   │
│ 2       │ '103'  │ '103 torso_left'   │ 1                                │ 'OK'   │
│ 3       │ '104'  │ '104 head_yaw'     │ 1                                │ 'OK'   │
└─────────┴────────┴────────────────────┴────────────────────────────────┴────────┘

=== Home offset apply summary ===
┌─────────┬────────┬────────────────┬────────────────┐
│ (index) │ Device │ Joint          │ Status         │
├─────────┼────────┼────────────────┼────────────────┤
│ 0       │ '104'  │ '104 head_yaw' │ 'APPLIED & SAVED' │
└─────────┴────────┴────────────────┴────────────────┘
Parameters were saved (0x1010) on every successfully applied device — they now persist across a power cycle.

Actual position for all devices:

┌─────────┬────────┬────────────────────┬─────────────────┐
│ (index) │ Device │ Joint              │ Position (deg)  │
├─────────┼────────┼────────────────────┼─────────────────┤
│ 0       │ '101'  │ '101 torso_yaw'    │ '2.15'          │
│ 1       │ '102'  │ '102 torso_right'  │ '-0.87'         │
│ 2       │ '103'  │ '103 torso_left'   │ '0.00'          │
│ 3       │ '104'  │ '104 head_yaw'     │ '-90.00'        │
└─────────┴────────┴────────────────────┴─────────────────┘

=== Stage 5: Apply software position limits ===

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

Note the 3 `MATCH` joints (101, 102, 103) don't appear in Stage 3 at all — only `MISMATCH`
devices are ever considered there. All 4 devices reappear in Stage 4's, Stage 5's, and Stage
7's tables, since all three cover every device that went through Stage 2. Device 104 was at
`+90°` before its polarity inverted in Stage 2, which (with no correction) would display as
`-450°`; Stage 3 reset its home offset to `0` and calculated `+360°`, and Stage 4 applied that,
landing the reported position at `-90°` — the negated original angle, as expected. Every device
in this run ends up saved: device 104 by Stage 4 (which also persists
its Stage 2 polarity write), every device by Stage 5 (which persists whatever's live on it at
that point, including any Stage 4 changes), and devices 101/103 additionally by Stage 7 (which
only touches devices whose quick-stop/SS1 values weren't already at target) — so nothing here
reverts on the next power cycle.

### If STO is active

If any device's Safe Torque Off is active when Stage 2 runs, the whole `--apply` run stops
immediately, before Stage 1's confirmed changes are written:

```
=== Stage 2: Apply polarity changes and controlword transition ===

[FATAL] STO (Safe Torque Off) is active on: 104 (104 head_yaw). The controlword transition 0 -> 6 -> 7 cannot succeed while STO is active. Release STO on the affected device(s) and re-run --apply.
```

Release STO on the named device(s) and re-run `--apply` — Stage 1 will need to be confirmed
again, since nothing from that run was written.

---

## Using this as a template

Nothing here is specific to one robot, but three things are specific to *a* robot and need
checking before the first `--apply` run.

### 1. Add your joint configs

Export a `config.csv` per drive and put them in `drive_configs/`. Any filename ending in `.csv`
is picked up; the directory ships empty apart from a `.gitkeep` note, which the scripts ignore.

A naming convention of `<number>_<joint_name>.config.csv` (e.g. `104_head_yaw.config.csv`) is
worth following: it makes the directory readable, and `load-drive-configs.ts` can fall back to
the filename when a file has no label row yet.

### 2. Label every drive and every config file

Matching is done on the `0x20F2:00` label, in the form `"<number> <joint name>"`:

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
| **Encoder resolution** | `ASSUMED_ENCODER_RESOLUTION_BITS` in `write-drive-config.ts` (20 bits) | Stages 3 and 4 convert between counts and degrees. A drive with a different resolution gets a wrong home offset correction, so set this to your encoders or make it per-joint. |
| **Polarity-inversion correction** | `HOME_OFFSET_ADJUSTMENT_DEGREES` (360°) | Inverting polarity at a non-zero angle shifts the actual position by a full revolution. Correct for a rotary joint; revisit for a linear axis. |
| **Quick stop / SS1 targets** | The Stage 7 constants | These are commissioned values, not defaults. Confirm them against your own safety concept before applying. |
| **SMM credentials** | `--username` / `--password` (default `Test` / `SomanetSMM`) | Stage 7 needs the real credentials for your drives. |

### 4. Bring up in this order

1. `assign-names.ts --dry-run`, then `assign-names.ts` — label every drive by slave position
   (see [assign-names.ts](#assign-namests)). Skip this if the drives already carry their labels.
2. `load-drive-configs.ts --dry-run` — check the pairing table before writing anything.
3. `load-drive-configs.ts --exclude ...` — load the configs, excluding per-drive calibration
   values (see below).
4. `write-drive-config.ts --diagnose` — read-only polarity check.
5. `write-drive-config.ts --apply` — staged, each stage confirmed.

---

## assign-names.ts

Labels every drive in one command, so a new robot can be named without one `0x20F2:00` write
per drive.

```
# Read-only: show which drive would get which label
npx ts-node src/humanoid_bringup/assign-names.ts --dry-run

# Assign every label (asks for "yes" first)
npx ts-node src/humanoid_bringup/assign-names.ts

# One slave position only
npx ts-node src/humanoid_bringup/assign-names.ts --slave-position 7
```

> **Applying this step assumes EtherCAT network is wired exactly same between the robots.**
> The script prints this as a large warning before the table and again before asking for
> confirmation.

Each file in `drive_configs/` is named `position_<NN>_<name>_config.csv`; `NN` is the slave
position it was exported from, and the file's own `0x20F2:00` row holds the label. The script
writes that label to the drive at the same position on the connected robot: `position_01_*` to
the 1st slave, `position_02_*` to the 2nd, and so on.

- **"Nth slave" is the Nth device in bus order**, not the raw position number, because Motion
  Master counts from 1 with SOEM and from 0 with IgH. Slaves that are not drives still count,
  which is why the file numbers have gaps.
- After each write the label is read back, then the drive's parameters are saved
  (`0x1010: Store parameters`), so the label survives a power cycle.
- Slaves with no file are left untouched. A file with no slave at its position, two files
  claiming one position, or two files with the same joint number is reported and skipped.
- It warns when the number of slaves on the network differs from the highest position in the
  files, which is the usual sign the wiring is not the same.

Check the table against the physical robot before typing `yes`: a drive at the wrong position
gets the wrong name, and every later step trusts that name.

---

## load-drive-configs.ts

Loads each joint's config file onto its drive. Matching is identical to
`write-drive-config.ts`, so both scripts agree about which file belongs to which joint.

```bash
# Every device, read-only
MOTION_MASTER_HOSTNAME=<host> npx ts-node src/humanoid_bringup/load-drive-configs.ts --dry-run

# Every device, keeping each drive's own commutation offset and encoder 2 configuration
MOTION_MASTER_HOSTNAME=<host> npx ts-node src/humanoid_bringup/load-drive-configs.ts \
  --exclude 0x2001:00,0x2112

# One joint — by joint number, serial number, or part of the joint name
MOTION_MASTER_HOSTNAME=<host> npx ts-node src/humanoid_bringup/load-drive-configs.ts \
  -d 104 --exclude 0x2001:00
```

### Options

| Option | Default | Meaning |
| --- | --- | --- |
| `-d, --device-ref <value>` | all labelled devices | Load one joint only. Accepts a joint number, a serial number, or part of the joint name. |
| `--exclude <ids>` | – | Objects to leave out of **every** joint. `0x2001:00` excludes that subindex; `0x2001` excludes every subindex of the object. |
| `--strategy <merge\|replace>` | `merge` | How each device's `config.csv` is updated — see below. |
| `--config-dir <path>` | `drive_configs/` | Directory holding the joint config files. |
| `--include-label` | off | Also write the joint label `0x20F2:00`. |
| `--dry-run` | off | Report what would be sent without writing. |

### What `--exclude` means depends on the strategy

| Strategy | An excluded object ends up |
| --- | --- |
| `merge` (default here) | **keeping the drive's current value** — Motion Master reads the existing `config.csv`, applies the file's rows on top, writes the result back |
| `replace` (the library's default) | **reverted** — `config.csv` is overwritten without that row, so `0x1011:01` falls back to `.factory_config` / `.assembly_config` |

So excluding an object only *preserves* it under `merge`. Under `replace` it is reset.

Good exclusion candidates are values belonging to one physical drive rather than to the machine
configuration: the commutation angle offset (`0x2001:00`), encoder offsets, the station alias.
Those are exactly what a shared config should not overwrite on a calibrated drive.

### Always excluded

- **SMM safety parameters.** The safety layer ignores plain writes to them and only accepts the
  authenticated, CRC-validated SMM configuration sequence, so sending them here would silently
  do nothing. Use `../Vesoma_SS1/configure-damping-parameters.promise.ts` for those, or Stage 7
  of `write-drive-config.ts`.
- **The joint label `0x20F2:00`**, unless `--include-label` is given. Loading it is harmless
  when the file matched the right drive, but it is the one object that must never reach the
  wrong one, and under `--strategy replace` a file without that row would leave a drive
  unlabelled — which would then match nothing on the next run.

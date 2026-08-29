# Calibrate Hand-Eye Geometry

> Category: How-to. The [Chinese source](../../zh/how-to/calibrate-hand-eye.md) is authoritative.

Hand-eye calibration determines the camera pose relative to the robot so image coordinates can be projected into the
robot base frame. Calibrate after the first installation, after moving the camera or bracket, or when grasp positions
show a stable directional offset.

The Piper workflow uses [calibrate_hand_eye.py](../../../scripts/calibrate/calibrate_hand_eye.py) and writes
`configs/piper/piper_calib.json`.

## Contents

Prepare the camera, board, and robot; collect diverse poses; solve and assess the transform; then verify it before use.

## 1. Prepare the system

### 1.1 Install dependencies

Install calibration and Piper dependencies:

```bash
pip install -e ".[calib,piper]"
```

`[calib]` provides OpenCV and RealSense support; `[piper]` provides the Piper arm driver (if your arm is not a Piper,
install the corresponding dependencies yourself).

### 1.2 Prepare the calibration board

Prepare a flat, matte ChArUco board (recommended) or chessboard. To generate a printable ChArUco image:

```bash
python scripts/calibrate/calibrate_hand_eye.py --generate-board board.png \
  --board charuco --squares-x 5 --squares-y 7 \
  --square-size-mm 30 --marker-size-mm 22
```

**Advice for a home-made board:**

1. Print on A4 at **100% original scale**, with "fit to page / scaling" turned off.
2. After printing, **measure the actual edge length of a large black square and of a small black square with a ruler**,
   and pass those real millimetre values to `--square-size-mm` and `--marker-size-mm` respectively. **Print scaling is
   the number-one source of error.**
3. **Mount it flat on a rigid backing** (foam board / acrylic / aluminium) — no bending, no curling.
4. Keep the surface **non-reflective** (matte paper is best).
5. Remember your board parameters (square count, square size, marker size) — **generation and calibration must use the
   same set**.

For the large black squares, measuring the total length of several squares along one row and dividing by the count is
more accurate than measuring a single square.

### 1.3 Configure the robot

Edit [scripts/calibrate/calibrate.yaml](../../../scripts/calibrate/calibrate.yaml) and set the RealSense serial:

```yaml
camera_serial: "your-camera-serial"
```

Or temporarily use an environment variable: `export CAMERA_SERIAL=your-camera-serial`.

> This configuration is purpose-built for calibration. **Do not** use `piper.yaml` — that one points at the calibration
> file currently being generated.

## 2. Start calibration in three steps

### Step 1: Run self-checks and confirm settings

Power the robot, bring up CAN, place the board fully inside the camera view, and run:

```bash
python scripts/calibrate/calibrate_hand_eye.py \
  --config scripts/calibrate/calibrate.yaml \
  --board charuco --squares-x 5 --squares-y 7 \
  --square-size-mm 30 --marker-size-mm 22
```

Replace both size values with your measurements.

### Step 2: Collect multiple viewing angles (most important)

The interactive collector uses these keys:

| Key | Action |
| --- | --- |
| Enter | Capture the current view |
| `s` | Stop capture and solve |
| `u` | Remove the most recent view |
| `q` | Abort without solving |

Collect 10–15 views. Translation alone is insufficient: vary wrist rotation by roughly ±20–30 degrees in several
directions, rotate about the optical axis, and vary distance while keeping the entire board sharp and visible. The UI
reports whether each frame was accepted and warns when rotational diversity is too small.

For automatic collection, add `--auto`. This moves the real robot: keep the E-stop within reach and inspect planned
poses first with `--auto-dry-run`.

### Step 3: Solve and write the file

After `s`, the solver prints a quality report and asks before replacing the output. An existing file is backed up with
a `.bak` suffix.

## 3. Accept or reject the solution

Use all three quality dimensions:

| Metric | Good | Recalibrate |
| --- | --- | --- |
| Reprojection RMS | below 1 px | above 2 px |
| Hand-eye consistency | below about 0.5° and 2–3 mm | materially larger |
| Board-origin consistency | standard deviation below 2–3 mm | above 3 mm |

Poor results usually mean that wrist rotation was insufficient or the printed square size was entered incorrectly.
Collect a more diverse set instead of compensating a bad transform with runtime offsets.

## 4. Verify on hardware

Use `--verify` for a zero-motion numerical projection check. For a physical alignment check, use `--verify-touch`; the
robot hovers the tool tip about 30 mm above the detected board center by default.

For a bare flange:

```bash
python scripts/calibrate/calibrate_hand_eye.py \
  --config scripts/calibrate/calibrate.yaml \
  --board charuco --squares-x 5 --squares-y 7 \
  --square-size-mm 30 --marker-size-mm 22 \
  --verify-touch
```

> When the configuration has `tool_offset_mm=0`, the program **warns loudly and asks you to confirm** that the end
> really carries no tool — because it computes the hover height treating "flange = tip".

If a gripper or tool is installed, provide the measured flange-to-tip distance:

```bash
python scripts/calibrate/calibrate_hand_eye.py ... \
  --verify-touch --verify-tool-offset-mm 95
```

Never omit the offset for an installed tool: the tip can move lower than expected and strike the board. Increase the
default clearance with `--verify-hover-mm`. Validate several positions across the working area before treating the
file as a deployment calibration.

## 5. Troubleshoot

The board is not detected:

Keep the complete board in frame, reduce tilt and distance, avoid reflections, and verify that the command uses the
same board dimensions as the printed target.

Camera intrinsics are unavailable:

RealSense normally supplies factory intrinsics. For another camera, add `--calibrate-intrinsics` or provide
`--intrinsics fx fy ppx ppy`.

Too few valid views:

At least three accepted views are required; 10–15 diverse views are recommended. Change pose more and take more shots.

Use a chessboard instead of ChArUco:

Replace `--board charuco --marker-size-mm ...` with `--board chessboard`; everything else is the same. ChArUco is more
robust to occlusion and blur, so prefer it.

No hardware is available:

Run the numerical self-test:

```bash
python scripts/calibrate/calibrate_hand_eye.py --selftest
```

## 6. Option reference

| Option | Meaning |
| --- | --- |
| `--config PATH` | Calibration-specific robot configuration |
| `--board {charuco,chessboard}` | Target type |
| `--squares-x`, `--squares-y` | Board grid dimensions |
| `--square-size-mm` | Measured square size |
| `--marker-size-mm` | Measured ChArUco marker size |
| `--auto` | Move the robot through automatic capture poses |
| `--calibrate-intrinsics` | Estimate camera intrinsics from the captured views |
| `--out PATH` | Output file; defaults to `configs/piper/piper_calib.json` |
| `--verify` | Print projected verification coordinates without motion |
| `--verify-touch` | Hover above the board center for physical verification |
| `--verify-tool-offset-mm` | Installed flange-to-tip distance for touch verification |
| `--verify-hover-mm` | Hover clearance, default 30 mm |
| `--generate-board PATH` | Generate a printable target image |
| `--selftest` | Run the solver against synthetic data |

Run `python scripts/calibrate/calibrate_hand_eye.py --help` for the complete current option set.

## SO-101 fixed-camera workflow

For the complete SO-101 eye-to-hand procedure—including hardware preparation, waypoint teaching, automatic
collection, offline re-solving, and runtime configuration—see
[Calibrate an SO-101 with a Fixed Camera](calibrate-so101-eye-to-hand.md).

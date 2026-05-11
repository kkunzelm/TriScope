# TriScope

TriScope — triangulation + scope (microscope): microscope live view, precision stage control, and laser triangulation scanner.

Desktop application for a motorised video measuring microscope. Combines live camera acquisition with precision stage control to enable dimensional measurements directly from the camera image or via stage displacement. Includes a laser-line triangulation scanner that produces PLY point clouds.

---

## Features

- Live camera feed — IDS Peak (GenICam) and V4L2 (USB/webcam) cameras
- XYZ stage control — Lang LStep 23 (MCL3 protocol) and DIY stepper stub
- Measurement overlays — distance, angle, radius with µm/pixel calibration
- Table measurement mode — read XY displacements between two points at stage precision
- Crosshair and 10×10 grid overlays
- Laser-line triangulation scanner — stop-and-go scan, Gaussian sub-pixel extraction, PLY export

---

## Prerequisites

| Dependency | Minimum version | Notes |
|---|---|---|
| CMake | 3.20 | |
| Qt | 6.4 | Core, Gui, Widgets, SerialPort |
| OpenCV | 4.x | core, imgproc, imgcodecs |
| Eigen3 | 3.3 | Header-only; used by the scanner triangulator |
| IDS Peak SDK | 2.x | Only needed for IDS GenICam cameras; set `IDS_PEAK_ROOT` if installed outside `/opt/ids-peak` |
| Linux kernel | any recent | V4L2 support for USB cameras |

Install on Debian/Ubuntu:

```bash
sudo apt install cmake qt6-base-dev qt6-serialport-dev libopencv-dev libeigen3-dev
```

IDS Peak SDK must be downloaded separately from IDS Imaging and installed to `/opt/ids-peak` (default) or another path passed via `-DIDS_PEAK_ROOT=…` at configure time.

---

## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
```

Run via the generated wrapper script (sets `GENICAM_GENTL64_PATH` and library paths for IDS cameras):

```bash
./VideoMeasuringMicroscope.sh
```

Direct execution also works if IDS Peak runtime libraries are on `LD_LIBRARY_PATH`:

```bash
./TriScope
```

---

## Test ohne Hardware (Simulator)

Für Entwicklung und Tests ohne angeschlossenen Messtisch steht ein Python-Simulator zur Verfügung, der die wichtigsten MCL3-Befehle beantwortet.

**Terminal 1 — virtuelles Port-Paar und Simulator starten:**

```bash
./tools/run-sim.sh
# Gibt den GUI-Port aus, z. B.: GUI-Port: /tmp/lstep-app
```

**Terminal 2 — Anwendung starten:**

```bash
./build/TriScope
```

In der GUI **Aktualisieren** drücken. Der Port `/tmp/lstep-app` erscheint in der Dropdown-Liste (oder manuell eintragen). Anschließend **Verbinden** drücken.

Der Simulator beantwortet Positions-, Status-, Versions- und Bewegungsbefehle mit plausiblen Testwerten, reagiert aber nicht auf alle Spezialfälle (z. B. echte Endschalter-Ereignisse beim Tischhub-Messen).

---

## Application layout

The window is split into a **left panel** (tabs, max 420 px wide) and a persistent **live camera view** on the right. The camera view is always visible regardless of which tab is active.

| Tab | Contents |
|---|---|
| **Connect** | Camera discovery, exposure/gain, streaming — Stage port/type, Connect/Disconnect, Home (Calibrate), Measure Range |
| **Microscope** | Unit toggle, position display, ΔX/ΔY table measurement, jog controls, Go to Position, Abort — Crosshair/Grid — Distance/Angle/Radius overlay tools |
| **Scanner** | Stage jog + Go to Position for scanner positioning — Scan parameters — Camera settings — Output file — Progress and Start/Abort |
| **Calibrate** | Calibration parameters (y\_ref, scale\_z, scale\_y, cx) — Calibrate Z / Calibrate Y wizards — Save/Load JSON |

---

## Quick-start workflow

### 1 — Connect and start the camera *(Connect tab)*

1. Click **↺** to discover available devices.
2. Select the camera from the dropdown (auto-selected after refresh).
3. Click **Start Streaming**.
4. Adjust **Exposure** and **Gain** until the image is well-exposed.

### 2 — Connect the stage *(Connect tab)*

1. Select the serial port (e.g. `/dev/ttyUSB0`). Click **↺** to refresh the port list.
2. Select stage type: **LStep 23** or **DIY Stepper**.
3. Click **Connect**. The position display in the Microscope tab updates when the stage responds.

> **Shutdown:** click **Disconnect** before closing, or close the window — the application sends an abort command and closes the serial port cleanly either way.

### 3 — Home / Calibrate *(Connect tab)*

Click **Home (Calibrate)**.

All three axes drive to their home switches. This takes approximately 10–15 s. After calibration the software origin (0, 0, 0) is at the home-switch position. **Always calibrate after powering on the stage** — without it the displayed coordinates are meaningless.

### 4 — Set WCS origin *(Connect tab)*

After calibration the stage is at (0, 0, 0). If a different point should be the measurement origin, jog to that position and click **Set as Home**. This re-defines (0, 0, 0) to the current position without any movement.

### 5 — Measure travel range *(Connect tab)*

Click **Measure Range** once per session (after Calibrate). The axes drive to their far end-switches (~20–30 s) to record the full travel. The Z axis is normally negative during use (stage below the objective).

### 6 — Jog the stage *(Microscope or Scanner tab)*

Select a step size (0.001 mm – 50 mm) and click the ±X / ±Y / ±Z buttons. The Z axis uses a reduced speed and gentle ramp so the electromagnetic brake has time to disengage.

### 7 — Go to absolute position *(Microscope or Scanner tab)*

Enter X, Y, Z coordinates in the **Go to (mm)** spinboxes and click **Move to Position**. The spinboxes track the live stage position until you edit them; after a **Move to Position** they resume tracking.

### 8 — Pixel / µm calibration *(Microscope tab)*

Required before using the Distance / Angle / Radius overlay tools in real-world units.

1. Place a reference object of known size in the field of view.
2. Click **Distance** and mark the known dimension in the image.
3. Enter the known length (µm) and measured pixels, then click **Set**.

### 9 — Table measurement *(Microscope tab)*

Measures distances between two points using stage displacement rather than image pixels — sub-micron repeatability.

1. Enable **Crosshair** (Overlays section).
2. Jog until point A is at the crosshair centre.
3. Click **Set Origin (Zero ΔX/ΔY)** — ΔX and ΔY reset to 0.
4. Jog until point B is at the crosshair centre.
5. Read **ΔX** / **ΔY** — true stage displacement between A and B.

---

## Measurement overlay tools *(Microscope tab)*

| Tool | Clicks | Output |
|---|---|---|
| Distance | 2 | Length between points |
| Angle | 3 | Angle at the middle point |
| Radius | 3 (on arc) | Radius of the best-fit circle |
| Clear | button | Remove all annotations |

The result is shown as a label in the image and in the status bar. The next click after a completed measurement starts a fresh measurement of the same type. Click **Clear** or switch tools to stop.

---

## Scanner *(Scanner tab + Calibrate tab)*

The scanner projects a laser stripe across the object, moves the X stage in steps, captures one frame per step, extracts the laser centroid per camera column (sub-pixel Gaussian or centre-of-gravity), and writes a PLY point cloud.

### First-time setup

1. Open the camera and start streaming in the **Connect** tab.
2. Connect and calibrate the stage.
3. Switch to the **Scanner** tab.
4. Set **Exposure** (µs) so the laser line is bright but not saturated.
5. Click **Histogram…** to see the intensity distribution. Set **Threshold** just above the ambient-light floor so laser pixels pass and background pixels are rejected. The red line in the histogram shows the current threshold.
6. **Max scatter** (px) — columns whose laser-line row position deviates more than this from the median across the frame are rejected as noise. 20 px is a good starting point for a relatively flat surface (≈ 1 mm height variation with default scale\_z).
7. Calibrate the scanner geometry (see below).
8. Set **Start X**, **End X** (use **← Pos** to capture the current stage position), and **Step**.
9. Choose an output file with **Browse…**.
10. Click **Start Scan**.

During the scan the camera view updates with each grabbed frame so you can verify the laser line is visible. Click **Abort** to stop early; partial data is discarded.

### Stage controls during scanning

The **Stage** section of the Scanner tab lets you jog and position the stage independently of the Microscope tab. **Move to WCS Origin** drives to (0, 0, 0) — only enabled after a successful Calibrate or Set as Home; it shows an informational message if the WCS origin has not been set.

### Calibration *(Calibrate tab)*

The scanner uses a double-telecentric triangulation model (Weber 1995):

```
z_world = (y_ref − row_px)  × scale_z   [mm]
y_world = (col_px − cx)     × scale_y   [mm]
x_world = table_x                        [mm]
```

| Parameter | Meaning | Set by |
|---|---|---|
| `y_ref` (px) | Camera row where the laser appears at Z = 0 | Z calibration |
| `scale_z` (mm/px) | Depth per pixel of laser-line row shift | Z calibration |
| `scale_y` (mm/px) | Lateral mm per camera column | Y calibration |
| `cx` (px) | Camera column that maps to world Y = 0 | Y calibration |

**Z calibration (Calibrate Z…)**

1. Place a flat, diffuse surface (white paper, ceramic tile) in the laser plane.
2. Enter step size (e.g. 1 mm) and number of steps (e.g. 5). The stage moves downward (−Z) through the steps.
3. The wizard fits `row = y_ref − z / scale_z` by least squares and updates `y_ref` and `scale_z`.

**Y calibration (Calibrate Y…)**

1. Place an object of exactly known width in the laser plane so both edges are visible.
2. Enter the width in mm.
3. The wizard detects the two edge columns and computes `scale_y` and `cx`.

Calibration parameters persist only within the session. Use **Save JSON** to store them and **Load JSON** to restore them at the next session.

> **Note:** Scanner calibration is currently untested in practice. Run a test scan on a flat reference surface and verify that Z values are consistent across the field before trusting measurements.

---

## Coordinate system

Right-handed system, ISO 841 / G-code convention:

| Axis | Zero (after Calibrate) | Positive direction |
|---|---|---|
| X | Home switch | → right |
| Y | Home switch | → away from user |
| Z | Home switch (top of travel) | ↑ up towards objective |

Working Z positions are negative (stage below the objective zero point).

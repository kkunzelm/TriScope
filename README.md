# VideoMeasuringMicroscope

Desktop application for a motorised video measuring microscope. Combines live camera acquisition with precision stage control to enable dimensional measurements directly from the camera image or via stage displacement.

## Features

- Live camera feed — IDS Peak (GenICam) and V4L2 (USB webcam) cameras
- XYZ stage control — Lang LStep 23 (MCL3 protocol) and DIY stepper
- Measurement overlays — distance, angle, radius with µm/pixel calibration
- **Table measurement mode** — zero the stage position at a reference point and read the XY displacement to a second point with stage precision
- Crosshair and 10×10 grid overlays

---

## Build

### Prerequisites

| Dependency | Minimum version | Notes |
|---|---|---|
| CMake | 3.20 | |
| Qt | 6.4 | Widgets, SerialPort modules |
| OpenCV | 4.x | imgproc, imgcodecs |
| IDS Peak SDK | 2.x | Optional; only needed for IDS GenICam cameras |
| Linux kernel | any recent | V4L2 support for USB webcams |

### Build steps

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
```

Run via the wrapper script (sets up library paths):

```bash
./VideoMeasuringMicroscope.sh
```

---

## Quick-start workflow

### 1 — Connect and start the camera

1. In the **Camera** section, click **↺** to discover available devices.
2. Select the camera from the dropdown (auto-selected after refresh).
3. Click **Start Streaming**.
4. Adjust **Exposure** and **Gain** sliders until the image is well-exposed.

### 2 — Connect the stage

1. Select the serial port from the port dropdown (e.g. `/dev/ttyUSB0`).
2. Select stage type: **LStep 23** or **DIY Stepper**.
3. Click **Connect**. The stage sends its initialisation sequence automatically; the position display updates to show the current hardware position.

### 3 — Home / Calibrate

Click **Home (Calibrate)**.

All three axes drive to their home switches (the zero reference). This takes approximately 10–15 s. After calibration the software origin (0, 0, 0) corresponds to the home-switch position.

**Always calibrate after powering on the stage.** Without a calibration the displayed coordinates are meaningless.

### 4 — Set travel range (Measure Range)

After homing, click **Measure Range**.

All axes drive to their opposite end-switches (~20–30 s) to measure the full travel. The measured values are stored and used for coordinate display. Run this once per session, after Calibrate.

### 5 — Jog the stage

Select a step size (0.001 mm – 50 mm) and click the ±X / ±Y / ±Z buttons.

The Z axis uses reduced speed and a gentle ramp so that the electromagnetic holding brake has time to disengage before the motor accelerates. If Z does not move on the first click, try a slightly larger step.

### 6 — Pixel / µm calibration (for image measurements)

Required before using Distance / Angle / Radius overlays in real-world units.

1. Place a reference object of known size in the field of view.
2. Click **Distance** and mark the start and end of the known dimension on the image.
3. Enter the known real-world length in the **µm** spinbox and the measured pixel count in the **px** spinbox.
4. Click **Set**. All subsequent overlay measurements are shown in µm.

Alternatively, move the stage a known distance (e.g. 1.000 mm via a jog), mark the displacement of a feature in the image, and enter 1000 µm as the reference length.

### 7 — Table measurement (XY distances via stage)

This mode measures distances using stage movement rather than image pixels, giving sub-micron repeatability.

1. Enable the **Crosshair** overlay (Overlays section) so the image centre is marked.
2. Jog until the first point of interest is exactly at the crosshair centre.
3. Click **Set Origin (Zero ΔX/ΔY)** — the ΔX and ΔY displays reset to 0.
4. Jog until the second point of interest is at the crosshair centre.
5. Read **ΔX** and **ΔY** — these are the true stage displacements and equal the distance between the two points in X and Y.

The unit toggle (mm / µm) applies to both the absolute position display and the ΔX / ΔY display.

---

## Measurement overlay tools

| Tool | Input | Output |
|---|---|---|
| Distance | Click 2 points | Length between points |
| Angle | Click 3 points | Angle at the middle point |
| Radius | Click 3 points on an arc | Radius of the best-fit circle |
| Clear | Button | Remove all annotations |

---

## Longterm / planned

- **IDS camera hardware ROI** — expose a "Zoom In" control that uses the GenICam `Width`, `Height`, `OffsetX`, `OffsetY` nodes to crop the sensor region. This reduces USB bandwidth, increases frame rate on the selected region, and avoids software scaling artefacts.

---

## Coordinate system

Right-handed system, ISO 841 / G-code convention:

| Axis | Zero position | Positive direction |
|---|---|---|
| X | Home switch (left) | → right |
| Y | Home switch (front) | → back |
| Z | Home switch (top) | ↑ up (objective direction) |

After Calibrate + Measure Range the stage reports positions in mm. The Z axis is normally negative during use (stage below the objective).

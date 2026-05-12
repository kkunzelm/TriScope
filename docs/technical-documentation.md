# Technical Documentation — TriScope			

## 1. Purpose and Application Context

This application controls a motorised video measuring microscope that has been extended into a **laser-line triangulation 3D scanner optimised for dental applications**. The primary use case is high-precision, high-resolution surface scanning of dental objects — crowns, bridges, implant abutments, bite splints — where sub-millimetre geometric accuracy is required for quality control, CAD/CAM fit verification, and archiving.

The optical setup uses a **double-telecentric lens** on the camera side, which eliminates perspective distortion across the field of view. Combined with a calibrated laser triangulation geometry, this yields repeatable depth measurements that are independent of lateral object position within the field — a property that conventional camera lenses with perspective projection cannot provide.

The scanner is built around existing laboratory infrastructure: a motorised XYZ microscope stage (Uhl MS4 with Lang LStep 23 controller) that provides 2.5 µm positional accuracy, an industrial monochrome camera (IDS Imaging), and a line laser (Rodenstock). The software integrates these three components into a stop-and-go scan loop and exports point clouds in PLY format for further processing in tools such as CloudCompare or Geomagic.

---

## 2. Hardware Architecture

### 2.1 XYZ Stage — Lang LStep 23 (MCL3 protocol)

The stage communicates over RS-232 serial (USB-serial adapter in practice) using the proprietary MCL3 binary protocol. Key properties:

- Hardware resolution: 1 µm (1 unit = 1 µm in the controller register)
- Null switches at hardware origin (0, 0, 0) — all three axes
- Z axis has an electromagnetic (EM) holding brake that engages when the motor is de-energised
- X/Y travel: ~165 mm each; Z travel: ~104 mm (MS4-WT02 hardware defaults)

The software abstracts this behind the `IPositioningStage` interface. All movement commands are non-blocking; completion is signalled asynchronously via `movementFinished()`.

### 2.2 Camera — IDS Imaging (GenICam / IDS Peak SDK)

Industrial monochrome camera accessed via the IDS Peak SDK (GenICam standard). The SDK delivers frames through a buffer queue. The application also supports **V4L2 cameras** (USB webcams, frame grabbers) as a lower-cost alternative for the microscopy view, though the double-telecentric scanner geometry assumes a fixed, well-characterised optic.

Acquisition runs in a dedicated `AcquisitionThread` (high-priority QThread) that calls `grabFrame()` in a loop and emits `frameReady(QImage)` to the GUI. During scanning, the acquisition thread is stopped and the scanner grabs frames directly (stop-and-go mode).

### 2.3 DIY Stage Option

The `IPositioningStage` interface is intentionally hardware-agnostic. A second implementation, `DIYStepperStage`, serves as the stub for a **DIY XYZ stage built from 3D-printer or CNC controller hardware**:

- Standard 3D-printer controller boards (Marlin firmware, RAMPS 1.4, SKR, Duet) expose a G-code serial interface that maps naturally onto `moveAbsolute` / `moveRelative` / `abort`
- CNC controller boards (GRBL, Mach3 USB) offer the same interface
- Linear rails + stepper motors from the 3D-printer ecosystem (e.g. LDO, Bondtech) achieve 5–10 µm repeatability at a fraction of the cost of a commercial microscope stage
- For a dental scanner application the relevant travel is 60–100 mm in X (scan axis), 20–40 mm in Y (object width), 10–20 mm in Z (depth of focus range)

To activate the DIY stage, implement the eight pure-virtual methods in `DIYStepperStage` using `QSerialPort` to send G-code commands. The UI, calibration wizards, and scanner loop require no changes — they interact exclusively through `IPositioningStage`.

---

## 3. Coordinate System

Right-handed system, ISO 841 / G-code convention:

| Axis | Null-switch position (after Calibrate) | Positive direction |
|---|---|---|
| X | Home switch | → right (scan direction) |
| Y | Home switch | → away from user |
| Z | Home switch (top of travel, near objective) | ↑ upward |

**Working coordinates are negative.** After `calibrate()`, the software origin (0, 0, 0) is placed 2 mm away from each null switch (to release the Z EM brake and provide a safety margin). All actual working positions have negative X, Y, and Z values.

The coordinate transform between hardware and software is:

```
swX = m_hwRef.x − hwX
swY = m_hwRef.y − hwY
swZ = m_hwRef.z − hwZ
```

After calibration: `m_hwRef = {2.0, 2.0, 2.0}`. The null switches map to `sw = (+2, +2, +2)`, so there are only 2 mm of positive travel before the switches — all useful travel is negative.

---

## 4. Homing and Coordinate Initialisation

### 4.1 `calibrate()` — physical homing sequence

1. **Phase 1** — MCL3 'c' command drives all axes to null switches at Speed=100, Ramp=50 (gentle ramp to avoid missing the switch). Controller sets its internal position registers to hw=(0,0,0). Z EM brake fully engages.

2. **Phase 2** — Z only, Speed=20 Ramp=200: moves Z +2 mm hardware (away from null switch, downward in software). This releases the EM brake with a slow enough ramp to allow current to re-energise the motor.

3. **Phase 3** — X+Y only, Speed=50 Ramp=500: moves X and Y +2 mm hardware.

4. `m_hwRef = {2, 2, 2}`, `m_position = {0, 0, 0}`.

After calibration the stage is safe to move in all directions; the null switches are 2 mm away.

### 4.2 `setHome()` — software-only origin redefinition

`setHome()` adjusts `m_hwRef` so that the **current physical position** maps to software (0, 0, 0), without any movement:

```cpp
m_hwRef = { m_hwRef.x − m_position.x,
            m_hwRef.y − m_position.y,
            m_hwRef.z − m_position.z };
m_position = {0, 0, 0};
```

This is the lightweight alternative to a full calibration when the stage has not been power-cycled and a new measurement origin is needed.

### 4.3 Difference: `setHome()` vs. `Set Origin (Zero ΔX/ΔY)`

| | Set Origin (ΔX/ΔY) | Set as Home |
|---|---|---|
| Hardware movement | None | None |
| Affects stage coordinate system | No | Yes — redefines (0,0,0) |
| Scope | Display layer only | LStepStage (all subsequent moves) |
| Axes | X and Y only | X, Y, and Z |
| Typical use | Measure distance between two table positions | Start work from a known reference without full calibration |

`Set Origin` is a pure display feature — it stores the current X/Y into `m_originX/m_originY` in the sidebar and shows ΔX/ΔY deltas. Nothing is sent to the controller.

### 4.4 `measureLength()` — travel range measurement

Drives all axes to their far end-switches to record actual travel (`m_hwRange`). This is **optional** in current use — `travelRange()` is not called anywhere in the UI or scan logic, and `m_hwRange` has correct hardcoded defaults for the MS4-WT02. Run it once after installing new hardware; do not run it routinely.

### 4.5 Reconnect without power cycle

If the controller is not power-cycled, its position registers retain their values across a software reconnect. `sendInitSequence()` reads those registers via `enqueuePositionQuery()` and reconstructs `m_position` correctly — no re-homing needed. However, `m_hwRef` resets to its constructor default `{165.8, 166.5, 0.0}` on reconnect, shifting the software coordinate frame by ~163 mm on X/Y compared to a post-calibrate session. Saved absolute coordinates from a calibrated session are not reusable in a reconnected-only session.

---

## 5. Scanner — Laser-Line Triangulation

### 5.1 Optical model

The scanner uses the **double-telecentric laser triangulation model** from Weber (1995). The observation optics (camera lens) are double-telecentric: parallel rays enter the lens regardless of the lateral object position, so there is no perspective distortion and `scale_y` is constant across the full field width.

The laser projects a line in the scene YZ-plane. The camera observes it from triangulation angle Θ. The table (X axis) moves the object through the laser plane.

Forward projection (pixel → world):

```
z_world = (y_ref  − row_px) × scale_z    [mm]
y_world = (col_px − cx)     × scale_y    [mm]
x_world = table_x_mm
```

Physical interpretation of the scale factors:

```
scale_z = pixel_pitch / (β × sin Θ)    [mm/px]
scale_y = pixel_pitch / β              [mm/px]
```

where β is the lens magnification and Θ is the triangulation angle. These are **not entered directly** — they are determined empirically by the calibration wizards, which is preferable because β and Θ are difficult to measure independently.

The hardcoded `theta_rad = 0.436332` (25°) is stored in the saved JSON for documentation only. It is not used in any computation.

### 5.2 Laser line extraction

For each camera column, the extractor finds the sub-pixel row position of the laser line centroid.

**Step 1 — find peak row:** scan all rows in the column; record `peakRow` and `peakVal`. If `peakVal ≤ threshold`, skip (no laser in this column).

**Step 2a — Gaussian 3-point log-interpolation (default, Weber 1995):**

```
i0 = pixel(peakRow−1, col),  i1 = pixel(peakRow, col),  i2 = pixel(peakRow+1, col)
denom = log(i0) − 2·log(i1) + log(i2)
row_subpx = peakRow + 0.5 · (log(i0) − log(i2)) / denom
```

Valid when `denom < −1e-10` and all three pixels are positive. Achieves approximately 1/20-pixel precision on a well-formed Gaussian laser profile.

**Step 2b — Centre-of-Gravity fallback:** used when Gaussian fails (peak at frame edge, saturated profile, or asymmetric beam). Weighted average of intensity × row, restricted to ±`windowRows` around `peakRow`.

**Post-filter — median outlier rejection:** after all columns are processed, the median row position across valid columns is computed. Any column whose position deviates more than `medianRejectRows` pixels from the median is invalidated. This removes hot-pixel hits and specular reflections that survive the threshold but are spatially inconsistent with the laser line.

`medianRejectRows` is in pixel units. With `scale_z = 0.05 mm/px`, 1 mm of surface height = 20 pixels. Recommended values:

| Surface | medianRejectRows |
|---|---|
| Flat reference (glass, tile) | 10–20 px |
| Moderate relief (dental crown) | 30–50 px |
| Large steps or grooves | 100 px or more |

### 5.3 Calibration parameters

| Parameter | Unit | Meaning | Calibrated by |
|---|---|---|---|
| `y_ref` | px | Camera row where laser appears at Z = 0 | Z wizard |
| `scale_z` | mm/px | Depth per pixel of row shift | Z wizard |
| `scale_y` | mm/px | Lateral mm per camera column | Y wizard |
| `cx` | px | Camera column mapping to world Y = 0 | Y wizard |

### 5.4 Z calibration wizard

Moves the stage through N steps of size Δz downward (−Z) while a flat diffuse surface sits under the laser. At each step one frame is grabbed and the mean valid laser row is recorded.

Least-squares linear regression on the collected `(z_mm, row_mean)` pairs:

```
model:    row = a + b·z    where  a = y_ref,  b = −1 / scale_z
→  scale_z = −1 / b
   y_ref   = a
```

**Sign behaviour with negative working coordinates:** with z values such as `{−55, −56, −57}` mm, the regression is sign-agnostic. The result is a positive `scale_z` (the laser line moves to larger row numbers as Z decreases). `y_ref` is extrapolated to Z = 0 (the null switch position) and will be a large value well outside the sensor — for example:

```
y_ref ≈ row_at_working_z + z_working / scale_z
      ≈ 400 + (−55) / 0.05  =  −700 px
```

This is correct and expected. The projection formula gives the right Z:

```
z_world = (−700 − 400) × 0.05 = −55 mm  ✓
```

`scale_z` is fit directly from the regression slope and is accurate. `y_ref` uncertainty grows with extrapolation distance to Z = 0, but since scans measure relative surface height (not absolute Z from the null switch), this does not affect scan quality.

### 5.5 Y calibration wizard

One frame is grabbed with a reference object of known physical width W straddling the optical axis. The wizard detects the two edge columns from the steepest gradients in the laser profile row positions, then:

```
scale_y = W / (col_right − col_left)   [mm/px]
cx      = (col_left + col_right) / 2   [px]
```

Suitable calibration objects: gauge blocks, precision-ground slots, calibration bars with parallel edges at a certified distance.

### 5.6 Stop-and-go scan loop

The acquisition thread is stopped for the duration of a scan. The GUI receives `movementFinished()` from the stage thread; the scan slot `onStepReady()` then:

1. Calls `grabFreshFrame()` — flushes any buffered V4L2 frames (stale from before the move), then grabs one fresh frame with a 2 s timeout
2. Converts the frame to 8-bit (`Format_Grayscale8`)
3. Runs `extractLaserProfile()` with current threshold and scatter settings
4. Calls `projectTo3D()` to accumulate 3D points
5. Emits `previewFrameReady(img)` so the CameraView shows the grabbed frame at each step
6. Triggers the next relative X move

At scan end, `PlyWriter` writes the accumulated point cloud to a binary little-endian PLY file (3 × float32 per vertex, coordinates in mm).

### 5.7 Intensity threshold

The threshold spinbox (0–255, 8-bit) sets the minimum pixel value that qualifies as laser return. Use the **Histogram** dialog to choose a value: set the threshold just above the ambient-light floor so laser pixels pass and background pixels are rejected. The red line in the histogram shows the current threshold value.

Camera exposure for scanning is set independently of the live-view exposure (Scanner → Camera → Exposure). A short exposure avoids saturation of the laser stripe while keeping ambient light below threshold.

---

## 6. Calibration Model Limitations and Future Extension

### 6.1 Current model — double-telecentric, no perspective

The calibration model described above (Weber 1995, four-parameter: `y_ref`, `scale_z`, `scale_y`, `cx`) is valid **only for double-telecentric optics**. It assumes:

- Constant magnification β across the entire field of view
- No radial or tangential lens distortion
- A parallel-ray (telecentric) observation path so that `scale_y` is independent of lateral object position

These assumptions hold for the current double-telecentric microscope objective + tube lens combination.

### 6.2 Alternative optic — CMOS sensor with photo macro lens

If the double-telecentric objective is replaced by a **CMOS sensor with a conventional photographic macro lens** (e.g. a mirrorless camera body with a 100 mm macro lens, a USB CMOS sensor with a C-mount macro lens, or a Raspberry Pi camera with extension tubes), the telecentric assumption breaks down:

- The lens has **perspective projection**: objects closer to the optical axis appear larger than those at the periphery
- The lens has **radial distortion** (barrel or pincushion), especially pronounced in close-focus macro configurations
- `scale_y` is no longer constant — it varies with lateral position
- The simple four-parameter model produces systematic errors that grow toward the image periphery

In this case the calibration must be replaced with **Zhang's method** (Zhang 2000) implemented via OpenCV:

```cpp
// OpenCV camera calibration (Zhang's method)
// Inputs: chessboard or circle-grid images at multiple orientations
cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                    cameraMatrix, distCoeffs,
                    rvecs, tvecs);

// cameraMatrix = [fx  0  cx]    distCoeffs = [k1 k2 p1 p2 k3]
//                [ 0 fy  cy]
//                [ 0  0   1]

// Undistort each grabbed frame before laser line extraction:
cv::undistort(src, dst, cameraMatrix, distCoeffs);
```

After undistortion, the 3D projection requires replacing the simple linear formula with the full perspective model:

```
Y_world = (col_px − cx) / fx × Z_world   [mm, from perspective geometry]
```

where `fx` is the focal length in pixels and `Z_world` comes from the laser triangulation row shift as before. This requires knowing Z first to recover Y — the decoupled calibration of the current model no longer applies, and a joint intrinsic + extrinsic calibration of the camera-laser system is needed.

A practical approach for a macro-lens setup:

1. Calibrate the camera intrinsics (`cameraMatrix`, `distCoeffs`) using a printed chessboard pattern at multiple poses (OpenCV's `calibrateCamera`)
2. Calibrate the laser plane in camera coordinates using a flat reference at known depths (the current Z-wizard approach, applied after undistortion)
3. Combine the two to reconstruct world XYZ from (row, col, table_x)

OpenCV 4.x (already a dependency) provides all necessary functions: `findChessboardCorners`, `calibrateCamera`, `undistort`, and `solvePnP`.

---

## 7. Software Architecture

### 7.1 Module overview

```
src/
├── interfaces/
│   ├── ICameraDevice.h/.cpp    — Qt-compatible camera abstraction (open/close/grabFrame/setExposure/setGain)
│   └── IPositioningStage.h     — Stage abstraction (moveAbsolute/moveRelative/calibrate/setHome/abort)
│
├── camera/
│   ├── IDSPeakCamera           — IDS Imaging GenICam implementation (IDS Peak SDK)
│   ├── V4L2Camera              — Linux V4L2 implementation (USB webcams, frame grabbers)
│   └── CameraDiscovery         — enumerates both backends; returns a unified device list
│
├── stage/
│   ├── LStepStage              — Lang LStep 23, MCL3 binary protocol over QSerialPort
│   └── DIYStepperStage         — Stub for DIY G-code stage (3D printer / CNC board)
│
├── acquisition/
│   └── AcquisitionThread       — High-priority QThread; grabs frames continuously, emits frameReady(QImage)
│
├── scanner/
│   ├── interfaces/ICamera.h    — Internal scanner Frame struct (uint8_t[], width, height)
│   ├── processing/
│   │   ├── LaserLineExtractor  — extractLaserProfile(): Gaussian + CoG sub-pixel extraction, median filter
│   │   └── Triangulator        — projectTo3D(), calibrateFromZPoints(), calibrateFromYEdges()
│   └── export/
│       └── PlyWriter           — Binary little-endian PLY export (float32 xyz)
│
└── ui/
    ├── MainWindow              — Central splitter (tabs left, CameraView right), hardware wiring
    ├── CameraView              — Live frame display, measurement overlay, crosshair/grid
    ├── SidebarWidget           — Connect panel + Microscope panel (jog, measure, overlays)
    ├── MeasurementOverlay      — Distance/Angle/Radius annotation rendering
    └── ScannerTab              — Scanner panel + Calibrate panel; owns scan loop state machine
```

### 7.2 Threading model

| Thread | Runs | Communicates via |
|---|---|---|
| GUI thread | All Qt widgets, scan state machine (onStepReady), histogram | Qt signals (QueuedConnection) |
| AcquisitionThread | Continuous camera grab loop | `frameReady(QImage)` signal |
| StageThread (QThread) | All LStepStage serial I/O | `QMetaObject::invokeMethod`, `positionChanged` / `movementFinished` signals |

The V4L2 camera's `mmap` buffer queue is not thread-safe for concurrent callers. AcquisitionThread must be fully stopped (wait for `isGrabbing()` to return false) before the scan loop calls `grabFrame()` directly. This is enforced in `startScan()` and both calibration wizards.

### 7.3 WCS origin tracking

`IPositioningStage::isHomed()` returns true once a valid coordinate frame has been established for the session:
- Set to `true` by `calibrate()` completion and by `setHome()`
- Reset to `false` by `disconnect()`
- `DIYStepperStage` always returns `false` (not yet implemented)

The "Move to WCS Origin" button in the Scanner tab checks `isHomed()` before issuing `moveAbsolute(0, 0, 0)` and shows an informational message if the origin has not been set.

---

## 8. V4L2 Camera Notes

### 8.1 Terratec Grabby artifact

The Terratec Grabby USB frame grabber (and some similar devices) produce a green-tinted artifact band in the bottom ~20 rows of each frame due to a hardware timing issue in the VIDIOC_DQBUF / VIDIOC_QBUF cycle. The application silently crops the bottom 20 rows in `V4L2Camera::grabFrame()` to avoid this artifact appearing in scan data. Other V4L2 devices are unaffected; the crop can be adjusted in that function if a different device exhibits a similar artifact at a different row.

### 8.2 Stale frame flushing

V4L2 uses a kernel-managed ring buffer (typically 4 frames). After a stage move, the camera buffer may contain frames captured before the move completed. The `grabFreshFrame()` helper drains all available frames (calls `grabFrame(timeout=0)` in a loop until it returns `nullopt`) before capturing one fresh frame with the full 2 s timeout. This ensures that each scan step's 3D data is from a frame taken after the stage stopped.

---

## 9. Key References

- **Weber, C. (1995)** — *Triangulation* (Diplomarbeit, Hochschule München). Derivation of the double-telecentric laser triangulation model, Gaussian sub-pixel interpolation formula (Equation 4). Source of the `scale_z = pixel_pitch / (β · sin Θ)` relationship and the 3-point log-Gaussian interpolation.

- **Mehl, R. (1992)** — *Laser-Linie Triangulation* (Diplomarbeit). Earlier treatment of the same measurement geometry; useful background on the CoG fallback method.

- **Zhang, Z. (2000)** — *A Flexible New Technique for Camera Calibration*. IEEE Transactions on Pattern Analysis and Machine Intelligence 22(11). The reference method for calibrating perspective (non-telecentric) cameras with lens distortion. Relevant if the double-telecentric optic is replaced.

- **IDS Peak SDK** — GenICam-compatible SDK for IDS Imaging cameras. See `docs/IDS_Peak_SDK_Programmierhandbuch.md` for the API reference used by `IDSPeakCamera`.

- **Lang LStep MCL3 Protocol** — Binary serial protocol for the LStep 23 controller. See `docs/lstep-protocol-reference.md` and `docs/lstep-protocol-reference-exzerpt-für-laserscanner.md` for the command set used by `LStepStage`.

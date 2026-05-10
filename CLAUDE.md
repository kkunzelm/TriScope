# CLAUDE.md - Video Measuring Microscope Project

## Project Overview
A C++20 / Qt 6.4+ application for controlling a video measuring microscope. The system integrates hardware positioning (Lang LStep & DIY Stepper) with industrial camera acquisition (IDS Peak & V4L2) and metrology tools.

## Tech Stack
- **Language:** C++20
- **Framework:** Qt 6.4 (Widgets)
- **Vision:** OpenCV (Core data structures and basic processing)
- **APIs:** IDS Peak SDK (GenICam), V4L2 (Linux)

## Camera Implementation Details
- **Dynamic Selection:** Implement a pull-down (QComboBox) that dynamically detects:
  - All available **V4L2** devices.
  - All **IDS Imaging** GenICam devices via IDS Peak API.
- **Support:** Handle both Mono and Color sensor types correctly.
- **Controls:** Implement GUI elements for Exposure and Gain for the active camera.

## Technical Constraints & Design
- **Coordinate System:** Right-Handed System.
  - **Origin (0,0,0):** Left-Frontal corner (from user perspective).
  - **Z-Axis:** Increases UPWARDS (+Z moves away from stage/towards objective).
## UI & Metrology
  - **UI:** Sidebar-centric (ids_peak_cockpit style).
  - **Features:** Unit toggle (mm/µm), Crosshair/Grid toggle, Jogging (0.001mm to 50mm steps).
  - **Tools:** Distance, Angle, and Radius measurement overlays.
## Design Patterns
- **Concurrency:** Acquisition MUST run in a dedicated thread.
- **Abstraction:** Use `ICameraDevice` and `IPositioningStage` interfaces for hardware independence.

- **Persistence:** Not implemented yet, but use an Observer/Interface pattern to allow future Data Logging/Database integration.

## Build Requirements
- CMake 3.20+
- Qt 6.4 (Core, Gui, Widgets, Multimedia)
- IDS Peak SDK & OpenCV 4.x

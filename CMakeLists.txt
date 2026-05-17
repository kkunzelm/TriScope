cmake_minimum_required(VERSION 3.20)

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

project(TriScope VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# ---------- Qt6 ----------
find_package(Qt6 6.4 REQUIRED COMPONENTS Core Gui Widgets SerialPort)

# ---------- OpenCV ----------
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# ---------- Threads ----------
find_package(Threads REQUIRED)

# ---------- Eigen3 ----------
find_package(Eigen3 REQUIRED NO_MODULE)

# ---------- IDS Peak SDK ----------
set(IDS_PEAK_ROOT "/opt/ids-peak" CACHE PATH "IDS Peak SDK installation prefix")

list(PREPEND CMAKE_PREFIX_PATH
    "${IDS_PEAK_ROOT}"
    "${IDS_PEAK_ROOT}/lib/cmake"
    "${IDS_PEAK_ROOT}/lib/x86_64-linux-gnu/cmake"
)

find_package(ids_peak REQUIRED
    HINTS
        "${IDS_PEAK_ROOT}/lib/cmake"
        "${IDS_PEAK_ROOT}/lib/x86_64-linux-gnu/cmake/ids_peak"
)

# Optional: IDS Peak IPL for Bayer demosaicing / pixel conversion
find_library(IDS_PEAK_IPL_LIB
    NAMES ids_peak_ipl
    HINTS "${IDS_PEAK_ROOT}/lib" "${IDS_PEAK_ROOT}/lib/x86_64-linux-gnu"
)
if(IDS_PEAK_IPL_LIB)
    message(STATUS "IDS Peak IPL found: ${IDS_PEAK_IPL_LIB}")
endif()

# ---------- V4L2 (Linux kernel headers) ----------
find_path(V4L2_INCLUDE_DIR linux/videodev2.h
    HINTS /usr/include /usr/local/include
)
if(V4L2_INCLUDE_DIR)
    message(STATUS "V4L2 found: ${V4L2_INCLUDE_DIR}")
else()
    message(WARNING "V4L2 headers not found – V4L2 camera backend will be disabled")
endif()

# ---------- Sources ----------
set(SOURCES
    src/main.cpp
    src/interfaces/ICameraDevice.cpp
    src/interfaces/IPositioningStage.cpp
    src/camera/CameraDiscovery.cpp
    src/camera/IDSPeakCamera.cpp
    src/camera/V4L2Camera.cpp
    src/acquisition/AcquisitionThread.cpp
    src/stage/LStepStage.cpp
    src/stage/DIYStepperStage.cpp
    src/ui/MainWindow.cpp
    src/ui/CameraView.cpp
    src/ui/SidebarWidget.cpp
    src/ui/MeasurementOverlay.cpp
    src/ui/ScannerTab.cpp
    src/scanner/processing/LaserLineExtractor.cpp
    src/scanner/processing/Triangulator.cpp
    src/scanner/export/PlyWriter.cpp
    src/scanner/export/XvWriter.cpp
    src/io/ViffWriter.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/scanner
    ${OpenCV_INCLUDE_DIRS}
    ${IDS_PEAK_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::SerialPort
    ${OpenCV_LIBS}
    ids_peak
    Eigen3::Eigen
    Threads::Threads
)

if(IDS_PEAK_IPL_LIB)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${IDS_PEAK_IPL_LIB})
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_IDS_IPL)
endif()

if(V4L2_INCLUDE_DIR)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_V4L2)
endif()

# IDS Peak deployment: copies GenTL producers and sets runtime library paths
ids_peak_deploy(${PROJECT_NAME})

# Generate a start script that sets GENICAM_GENTL64_PATH
if(UNIX)
    ids_peak_generate_starter_script(${PROJECT_NAME})
endif()

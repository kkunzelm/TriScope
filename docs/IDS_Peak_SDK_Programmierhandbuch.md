# IDS Peak SDK – Programmierhandbuch für C++17

**Stand:** 2026-05 | **SDK-Version:** ids-peak 2.9.0 | **Sprache:** C++17

Dieses Dokument fasst alle praxisrelevanten Erkenntnisse aus dem `indi_ids_ccd`-Treiberprojekt zusammen. Ziel ist es, ein neues Projekt (z. B. Triangulations-Laserscanner) ohne Lernkurve starten zu können.

---

## Inhaltsverzeichnis

1. [Build-System (CMake)](#1-build-system-cmake)
2. [SDK-Initialisierung und Kamera-Enumeration](#2-sdk-initialisierung-und-kamera-enumeration)
3. [Gerät öffnen und NodeMap cachen](#3-gerät-öffnen-und-nodemap-cachen)
4. [GenICam-Knoten lesen und schreiben](#4-genicam-knoten-lesen-und-schreiben)
5. [Pixel-Format-Verwaltung](#5-pixel-format-verwaltung)
6. [Frame-Geometrie und ROI](#6-frame-geometrie-und-roi)
7. [Acquisition-Pipeline (DataStream + Buffer)](#7-acquisition-pipeline-datastream--buffer)
8. [Belichtungssteuerung](#8-belichtungssteuerung)
9. [Gain-Steuerung](#9-gain-steuerung)
10. [UserSet-Management](#10-userset-management)
11. [Exposure-Worker-Thread](#11-exposure-worker-thread)
12. [Pixel-Konvertierung (10/12-bit auf 16-bit)](#12-pixel-konvertierung-1012-bit-auf-16-bit)
13. [Multi-Kamera-Betrieb](#13-multi-kamera-betrieb)
14. [Kritische Fallstricke und Lösungen](#14-kritische-fallstricke-und-lösungen)
15. [Minimales Einzelbild-Beispiel](#15-minimales-einzelbild-beispiel)
16. [Empfehlungen für Laserscanner-Projekte](#16-empfehlungen-für-laserscanner-projekte)

---

## 1. Build-System (CMake)

### Minimale `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

project(mein_ids_projekt VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# IDS SDK-Pfad – anpassen falls abweichend
set(IDS_PEAK_ROOT "/opt/ids-peak" CACHE PATH "IDS peak SDK prefix")

list(PREPEND CMAKE_PREFIX_PATH
    "${IDS_PEAK_ROOT}"
    "${IDS_PEAK_ROOT}/lib/cmake"
    "${IDS_PEAK_ROOT}/lib/x86_64-linux-gnu/cmake"
)

find_package(Threads REQUIRED)
find_package(ids_peak REQUIRED
    HINTS
        "${IDS_PEAK_ROOT}/lib/cmake"
        "${IDS_PEAK_ROOT}/lib/x86_64-linux-gnu/cmake/ids_peak"
)

# Optional: IDS Peak IPL (Image Processing Library) für Demosaicing etc.
find_library(IDS_PEAK_IPL_LIB
    NAMES ids_peak_ipl
    HINTS "${IDS_PEAK_ROOT}/lib" "${IDS_PEAK_ROOT}/lib/x86_64-linux-gnu"
)

add_executable(mein_ids_projekt main.cpp)

target_include_directories(mein_ids_projekt PRIVATE
    ${IDS_PEAK_INCLUDE_DIRS}
    "${IDS_PEAK_ROOT}/include/ids-peak/peak-ipl"  # nur bei IPL-Nutzung
)

target_link_libraries(mein_ids_projekt PRIVATE
    ids_peak
    Threads::Threads
)

# WICHTIG: Kopiert GenICam-Producer und setzt Laufzeitpfade korrekt
ids_peak_deploy(mein_ids_projekt)

# Erzeugt Starter-Script das GENICAM_GENTL64_PATH setzt
if(UNIX)
    ids_peak_generate_starter_script(mein_ids_projekt)
endif()
```

### Build-Workflow

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
# Ausführen immer über das generierte Starter-Script:
./mein_ids_projekt_start.sh
```

**Hinweis:** Das Starter-Script setzt `GENICAM_GENTL64_PATH` – ohne dieses findet der GenTL-Layer die Kamera-Producer nicht.

---

## 2. SDK-Initialisierung und Kamera-Enumeration

```cpp
#include <peak/peak.hpp>
#include <iostream>
#include <mutex>

// SDK genau einmal initialisieren (thread-safe)
static std::mutex sdkInitMutex;
static bool sdkInitialized = false;

void initializeSDK()
{
    std::lock_guard<std::mutex> lock(sdkInitMutex);
    if (sdkInitialized)
        return;

    peak::Library::Initialize();
    sdkInitialized = true;
}

// Alle angeschlossenen Kameras auflisten
std::vector<std::string> enumerateCameras()
{
    auto &deviceManager = peak::DeviceManager::Instance();
    deviceManager.Update();  // Neuabtastung der Hardware

    std::vector<std::string> names;
    for (const auto &desc : deviceManager.Devices())
    {
        // Eindeutiger Name: Modell + Seriennummer
        names.push_back(desc->ModelName() + "-" + desc->SerialNumber());
        std::cout << "Gefunden: " << desc->ModelName()
                  << " SN=" << desc->SerialNumber()
                  << " Interface=" << desc->InterfaceType() << "\n";
    }
    return names;
}

// Wichtig: SDK beim Programmende wieder freigeben
// peak::Library::Close();  // in main() oder RAII-Wrapper
```

---

## 3. Gerät öffnen und NodeMap cachen

```cpp
#include <peak/peak.hpp>

// Kamera anhand von Modell+Seriennummer öffnen
std::shared_ptr<peak::core::Device> openCameraByName(const std::string &targetName)
{
    auto &dm = peak::DeviceManager::Instance();
    dm.Update();

    for (const auto &desc : dm.Devices())
    {
        std::string name = desc->ModelName() + "-" + desc->SerialNumber();
        if (name == targetName)
        {
            // Control-Zugriff für Vollzugriff (vs. ReadOnly für reine Überwachung)
            return desc->OpenDevice(peak::core::DeviceAccessType::Control);
        }
    }
    throw std::runtime_error("Kamera nicht gefunden: " + targetName);
}

// NodeMap ist die Hauptschnittstelle zu allen Kamera-Parametern
void setupNodeMap(const std::shared_ptr<peak::core::Device> &device)
{
    // Remote-Device NodeMap = alle GenICam-Knoten der Kamera
    auto nodeMap = device->RemoteDevice()->NodeMaps().at(0);

    // Häufig benötigte Knoten gleich cachen:
    auto widthNode   = nodeMap->FindNode<peak::core::nodes::IntegerNode>("Width");
    auto heightNode  = nodeMap->FindNode<peak::core::nodes::IntegerNode>("Height");
    auto fmtNode     = nodeMap->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat");
    auto expNode     = nodeMap->FindNode<peak::core::nodes::FloatNode>("ExposureTime");

    std::cout << "Sensor: " << widthNode->Maximum() << "x" << heightNode->Maximum() << "\n";
    std::cout << "Pixelformat: " << fmtNode->CurrentEntry()->SymbolicValue() << "\n";
    std::cout << "Belichtung: " << expNode->Value() << " µs\n";
}
```

---

## 4. GenICam-Knoten lesen und schreiben

### Knotentypen

| GenICam-Typ         | Peak C++ Klasse                              | Verwendung                        |
|---------------------|----------------------------------------------|-----------------------------------|
| Integer             | `peak::core::nodes::IntegerNode`             | Width, Height, OffsetX, PayloadSize |
| Float               | `peak::core::nodes::FloatNode`               | ExposureTime, Gain, BlackLevel    |
| Enumeration         | `peak::core::nodes::EnumerationNode`         | PixelFormat, GainSelector, UserSetSelector |
| Command             | `peak::core::nodes::CommandNode`             | AcquisitionStart, AcquisitionStop, UserSetLoad |
| Boolean             | `peak::core::nodes::BooleanNode`             | diverse Flags                     |
| String              | `peak::core::nodes::StringNode`              | DeviceID, Seriennummer            |

### Muster: Sicheres Lesen/Schreiben

```cpp
// Template-Helper für lazy Node-Caching (aus ids_node_cache.h)
template <typename NodeT>
std::shared_ptr<NodeT> getNode(
    std::shared_ptr<NodeT> &cache,
    const std::shared_ptr<peak::core::NodeMap> &nodeMap,
    const char *name)
{
    if (!cache && nodeMap)
        cache = nodeMap->FindNode<NodeT>(name);
    return cache;
}

// FloatNode sicher lesen
double readExposureTime(const std::shared_ptr<peak::core::NodeMap> &nm)
{
    auto node = nm->FindNode<peak::core::nodes::FloatNode>("ExposureTime");
    if (!node || !node->IsReadable())
        throw std::runtime_error("ExposureTime nicht lesbar");
    return node->Value();
}

// FloatNode sicher schreiben mit Bereichsprüfung
void setExposureTime(const std::shared_ptr<peak::core::NodeMap> &nm,
                     double microseconds)
{
    auto node = nm->FindNode<peak::core::nodes::FloatNode>("ExposureTime");
    if (!node || !node->IsWriteable())
        throw std::runtime_error("ExposureTime nicht schreibbar");

    microseconds = std::clamp(microseconds, node->Minimum(), node->Maximum());
    node->SetValue(microseconds);
}

// EnumerationNode: Eintrag nach Name setzen
void setPixelFormat(const std::shared_ptr<peak::core::NodeMap> &nm,
                    const std::string &formatName)
{
    auto node = nm->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat");
    node->SetCurrentEntry(formatName);
}

// CommandNode ausführen
void executeAcquisitionStart(const std::shared_ptr<peak::core::NodeMap> &nm)
{
    auto node = nm->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart");
    if (!node || !node->IsAvailable())
        throw std::runtime_error("AcquisitionStart nicht verfügbar");
    node->Execute();
    node->WaitUntilDone();  // Blockiert bis Kommando abgeschlossen
}

// Verfügbare Enumeration-Einträge auflisten
void listPixelFormats(const std::shared_ptr<peak::core::NodeMap> &nm)
{
    auto node = nm->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat");
    for (const auto &entry : node->Entries())
    {
        if (entry->IsAvailable())
            std::cout << "  " << entry->SymbolicValue() << "\n";
    }
}
```

### Wichtig: Zugriffsstatus prüfen

```cpp
// Nicht alle Knoten sind immer schreibbar (z. B. während Streaming)
auto node = nm->FindNode<peak::core::nodes::IntegerNode>("Width");

bool readable  = node->IsReadable();
bool writeable = node->IsWriteable();
bool available = node->IsAvailable();

// Für Enumeration-Einträge:
for (const auto &entry : enumNode->Entries())
{
    auto status = entry->AccessStatus();
    if (status == peak::core::nodes::NodeAccessStatus::NotAvailable ||
        status == peak::core::nodes::NodeAccessStatus::NotImplemented)
        continue;  // Überspringen
}
```

---

## 5. Pixel-Format-Verwaltung

IDS-Kameras unterstützen verschiedene Formate. Für Laserscanner sind Mono-Formate relevant.

### Format-Erkennung

```cpp
struct PixelFormatInfo
{
    std::string idsName;       // z. B. "Mono12"
    int sourceBitDepth;        // 8, 10, 12, 16
    bool packed;               // "Mono12p" = packed
    uint8_t outputBitsPerPixel; // 8 oder 16 (nach Expansion)
};

// Bit-Tiefe aus Formatnamen extrahieren
int detectBitDepth(const std::string &formatName)
{
    if (formatName.find("16") != std::string::npos) return 16;
    if (formatName.find("12") != std::string::npos) return 12;
    if (formatName.find("10") != std::string::npos) return 10;
    return 8;
}

// Alle verfügbaren Mono-Formate ermitteln
std::map<std::string, PixelFormatInfo> queryMonoFormats(
    const std::shared_ptr<peak::core::NodeMap> &nm)
{
    std::map<std::string, PixelFormatInfo> result;
    auto node = nm->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat");

    for (const auto &entry : node->Entries())
    {
        if (!entry->IsAvailable())
            continue;

        const std::string name = entry->SymbolicValue();
        if (name.find("Mono") != 0)
            continue;  // Nur Mono-Formate für Laserscanner

        PixelFormatInfo info;
        info.idsName = name;
        info.sourceBitDepth = detectBitDepth(name);
        info.packed = !name.empty() && name.back() == 'p';
        info.outputBitsPerPixel = (info.sourceBitDepth <= 8) ? 8 : 16;
        result[name] = info;
    }
    return result;
}
```

### Empfohlene Formate für Laserscanner

| Format   | Bits | Payload/Pixel | Empfehlung                                    |
|----------|------|---------------|-----------------------------------------------|
| Mono8    | 8    | 1 Byte        | Schnell, ausreichend für starke Laserlinien   |
| Mono10   | 10   | 2 Byte        | Mehr Dynamik, direkt nutzbar                  |
| Mono12   | 12   | 2 Byte        | Optimal: 4096 Graustufen, gute Dynamik        |
| Mono12p  | 12   | 1.5 Byte      | Kleinerer Transfer, aber Entpacken nötig      |
| Mono16   | 16   | 2 Byte        | Maximale Dynamik, falls Sensor es unterstützt |

---

## 6. Frame-Geometrie und ROI

### ROI setzen (GenICam-Reihenfolge beachten!)

```cpp
// KRITISCH: Reihenfolge beim ROI-Setzen
// Viele GenICam-Kameras erfordern:
//   1. Offsets auf Minimum setzen
//   2. Dimensionen auf Minimum setzen
//   3. Neue Dimensionen setzen
//   4. Neue Offsets setzen
void setROI(const std::shared_ptr<peak::core::NodeMap> &nm,
            int64_t x, int64_t y, int64_t width, int64_t height)
{
    auto wNode  = nm->FindNode<peak::core::nodes::IntegerNode>("Width");
    auto hNode  = nm->FindNode<peak::core::nodes::IntegerNode>("Height");
    auto oxNode = nm->FindNode<peak::core::nodes::IntegerNode>("OffsetX");
    auto oyNode = nm->FindNode<peak::core::nodes::IntegerNode>("OffsetY");

    // Schritt 1+2: Offsets und Dimensionen zuerst auf Minimum
    oxNode->SetValue(oxNode->Minimum());
    oyNode->SetValue(oyNode->Minimum());
    wNode->SetValue(wNode->Minimum());
    hNode->SetValue(hNode->Minimum());

    // Schritt 3+4: Gewünschte Werte setzen
    wNode->SetValue(width);
    hNode->SetValue(height);
    oxNode->SetValue(x);
    oyNode->SetValue(y);
}

// Wert auf Inkrement normalisieren (Kamera akzeptiert nur Vielfache)
int64_t normalizeValue(int64_t value, int64_t minimum, int64_t increment)
{
    const int64_t safeInc = std::max<int64_t>(1, increment);
    if (value < minimum) return minimum;
    return ((value - minimum) / safeInc) * safeInc + minimum;
}

// Sensorgröße auslesen
std::pair<int,int> getSensorSize(const std::shared_ptr<peak::core::NodeMap> &nm)
{
    auto w = nm->FindNode<peak::core::nodes::IntegerNode>("Width");
    auto h = nm->FindNode<peak::core::nodes::IntegerNode>("Height");
    return {static_cast<int>(w->Maximum()), static_cast<int>(h->Maximum())};
}

// Pixelgröße auslesen (für Kalibrierung wichtig!)
std::pair<float,float> getPixelSize(const std::shared_ptr<peak::core::NodeMap> &nm)
{
    try {
        auto pw = nm->FindNode<peak::core::nodes::FloatNode>("SensorPixelWidth");
        auto ph = nm->FindNode<peak::core::nodes::FloatNode>("SensorPixelHeight");
        return {static_cast<float>(pw->Value()), static_cast<float>(ph->Value())};
    } catch (...) {
        return {0.0f, 0.0f};  // Nicht alle Kameras unterstützen dies
    }
}
```

### ROI für Laserscanner-Optimierung

```cpp
// Für Triangulations-Scanner: Nur den Bereich der Laserlinie aufnehmen
// Reduziert Datenvolumen und erhöht Framerate erheblich
void setLaserLineROI(const std::shared_ptr<peak::core::NodeMap> &nm,
                     int laserLineY,    // Zeile der Laserlinie (Mitte)
                     int roiHeight,     // Höhe des ROI (z. B. 50 Pixel)
                     int sensorWidth,
                     int sensorHeight)
{
    auto wNode  = nm->FindNode<peak::core::nodes::IntegerNode>("Width");
    auto hNode  = nm->FindNode<peak::core::nodes::IntegerNode>("Height");
    auto oxNode = nm->FindNode<peak::core::nodes::IntegerNode>("OffsetX");
    auto oyNode = nm->FindNode<peak::core::nodes::IntegerNode>("OffsetY");

    int64_t roiY = std::max(0, laserLineY - roiHeight / 2);
    roiY = normalizeValue(roiY, oyNode->Minimum(), oyNode->Increment());

    int64_t roiH = normalizeValue(roiHeight,
                                  hNode->Minimum(), hNode->Increment());

    // Volle Breite für den Laserstrahl
    int64_t roiW = normalizeValue(sensorWidth,
                                  wNode->Minimum(), wNode->Increment());

    // Reihenfolge beachten (siehe oben)
    oxNode->SetValue(oxNode->Minimum());
    oyNode->SetValue(oyNode->Minimum());
    wNode->SetValue(wNode->Minimum());
    hNode->SetValue(hNode->Minimum());

    wNode->SetValue(roiW);
    hNode->SetValue(roiH);
    oxNode->SetValue(0);
    oyNode->SetValue(roiY);
}
```

---

## 7. Acquisition-Pipeline (DataStream + Buffer)

Dies ist der Kern der Bildaufnahme. Der vollständige Lifecycle:

```
Initialize → AllocateBuffers → Lock TLParams → StartDataStream →
AcquisitionStart → WaitForFinishedBuffer → CopyData → QueueBuffer →
AcquisitionStop → StopDataStream → Unlock TLParams → RevokeBuffers
```

### Vollständige Implementierung

```cpp
#include <peak/peak.hpp>
#include <vector>
#include <chrono>

class IDSCamera
{
public:
    std::shared_ptr<peak::core::Device>    device;
    std::shared_ptr<peak::core::NodeMap>   nodeMap;
    std::shared_ptr<peak::core::DataStream> dataStream;
    size_t payloadSize = 0;

    // Schritt 1: DataStream öffnen und Buffer allozieren
    void setupAcquisition()
    {
        // DataStream öffnen (Index 0 = erster Stream)
        dataStream = device->DataStreams().at(0)->OpenDataStream();

        // PayloadSize aus NodeMap lesen (muss nach Format/ROI-Änderung neu gelesen werden!)
        auto psNode = nodeMap->FindNode<peak::core::nodes::IntegerNode>("PayloadSize");
        payloadSize = static_cast<size_t>(psNode->Value());

        // Mindestanzahl Buffer laut SDK
        size_t numBuffers = std::max<size_t>(
            3, static_cast<size_t>(dataStream->NumBuffersAnnouncedMinRequired()));

        for (size_t i = 0; i < numBuffers; ++i)
        {
            auto buffer = dataStream->AllocAndAnnounceBuffer(payloadSize, nullptr);
            dataStream->QueueBuffer(buffer);
        }
    }

    // Schritt 2: Acquisition starten
    void startAcquisition()
    {
        // TLParamsLocked = 1 muss VOR StartAcquisition gesetzt werden
        // Verhindert Parameter-Änderungen während des Streamings
        try {
            auto tlLock = nodeMap->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked");
            if (tlLock->IsWriteable())
                tlLock->SetValue(1);
        } catch (...) {}

        dataStream->StartAcquisition();  // KEIN Framecount-Argument!

        // Hardware-Akquisition starten
        auto startCmd = nodeMap->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart");
        startCmd->Execute();
        startCmd->WaitUntilDone();
    }

    // Schritt 3: Einzelbild aufnehmen
    std::vector<uint8_t> captureFrame(uint64_t timeoutMs = 5000)
    {
        auto buffer = dataStream->WaitForFinishedBuffer(timeoutMs);
        if (!buffer)
            throw std::runtime_error("Timeout beim Warten auf Bild");

        // Daten kopieren BEVOR Buffer zurückgegeben wird
        std::vector<uint8_t> frameData(
            static_cast<const uint8_t*>(buffer->BasePtr()),
            static_cast<const uint8_t*>(buffer->BasePtr()) + buffer->Size());

        // Buffer sofort zurückstellen für nächstes Bild
        dataStream->QueueBuffer(buffer);

        return frameData;
    }

    // Schritt 4: Acquisition stoppen
    void stopAcquisition()
    {
        // WICHTIG: KillWait() VOR StopAcquisition rufen!
        // Gibt blockierende WaitForFinishedBuffer()-Aufrufe frei
        try { dataStream->KillWait(); } catch (...) {}

        if (dataStream->IsGrabbing())
            dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);

        // TLParamsLocked entsperren
        try {
            auto tlLock = nodeMap->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked");
            if (tlLock->IsWriteable())
                tlLock->SetValue(0);
        } catch (...) {}

        // AcquisitionStop-Kommando
        try {
            auto stopCmd = nodeMap->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop");
            if (stopCmd && stopCmd->IsAvailable()) {
                stopCmd->Execute();
                stopCmd->WaitUntilDone();
            }
        } catch (...) {}
    }

    // Schritt 5: Buffer freigeben
    void freeBuffers()
    {
        try { dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll); } catch (...) {}

        // Alle angekündigten Buffer revoken
        auto buffers = dataStream->AnnouncedBuffers();
        for (auto &buf : buffers)
            dataStream->RevokeBuffer(buf);
    }
};
```

### Interruptibler Buffer-Wait (für abortierbare Belichtungen)

```cpp
// Wartet in kurzen Scheiben statt einem langen Timeout
// Ermöglicht sauberes Abbrechen von laufenden Belichtungen
std::shared_ptr<peak::core::Buffer> waitForBufferInterruptible(
    const std::shared_ptr<peak::core::DataStream> &ds,
    uint64_t totalTimeoutMs,
    uint64_t sliceTimeoutMs,
    const std::function<bool()> &shouldAbort)
{
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(totalTimeoutMs);

    while (std::chrono::steady_clock::now() < deadline)
    {
        if (shouldAbort && shouldAbort())
            return nullptr;

        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        if (remaining <= 0) break;

        const auto thisWait = std::min<uint64_t>(sliceTimeoutMs,
                                                  static_cast<uint64_t>(remaining));
        try
        {
            auto buffer = ds->WaitForFinishedBuffer(thisWait);
            if (buffer)
                return buffer;
        }
        catch (const std::exception &) {}  // Timeout ist eine Exception im IDS SDK!
    }

    return nullptr;
}
```

---

## 8. Belichtungssteuerung

```cpp
constexpr double US_PER_SECOND = 1'000'000.0;

// Belichtungszeit in Sekunden setzen
bool setExposureSeconds(const std::shared_ptr<peak::core::NodeMap> &nm,
                        double seconds)
{
    // Manche Kameras (ältere uEye-Kompatibilität) verwenden "ExposureTimeAbs"
    std::shared_ptr<peak::core::nodes::FloatNode> expNode;
    try {
        expNode = nm->FindNode<peak::core::nodes::FloatNode>("ExposureTime");
    } catch (...) {
        try {
            expNode = nm->FindNode<peak::core::nodes::FloatNode>("ExposureTimeAbs");
        } catch (...) {
            return false;
        }
    }

    const double minUs = expNode->Minimum();
    const double maxUs = expNode->Maximum();
    const double targetUs = seconds * US_PER_SECOND;

    if (targetUs < minUs || targetUs > maxUs)
        return false;

    expNode->SetValue(targetUs);
    return true;
}

// Belichtungsgrenzen auslesen
struct ExposureLimits {
    double minSeconds, maxSeconds, stepSeconds;
};

ExposureLimits getExposureLimits(const std::shared_ptr<peak::core::nodes::FloatNode> &expNode)
{
    ExposureLimits lim;
    lim.minSeconds  = expNode->Minimum() / US_PER_SECOND;
    lim.maxSeconds  = expNode->Maximum() / US_PER_SECOND;
    lim.stepSeconds = expNode->HasConstantIncrement()
                      ? expNode->Increment() / US_PER_SECOND
                      : 0.001;
    return lim;
}
```

### UserSet für erweiterten Belichtungsbereich

Einige IDS-Kameras haben zwei UserSets: `Default` (kurze Belichtung, hohe FPS) und `LongExposure` (lange Belichtung, niedrige FPS). Der Treiber implementiert automatisches Umschalten:

```cpp
// UserSet laden (aktiviert erweiterten Belichtungsbereich)
bool loadUserSet(const std::shared_ptr<peak::core::NodeMap> &nm,
                 const std::string &userSetName)  // "Default" oder "LongExposure"
{
    try {
        auto selector = nm->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector");
        auto loadCmd  = nm->FindNode<peak::core::nodes::CommandNode>("UserSetLoad");

        selector->SetCurrentEntry(userSetName);
        loadCmd->Execute();
        loadCmd->WaitUntilDone();
        return true;
    } catch (const std::exception &e) {
        std::cerr << "UserSet load failed: " << e.what() << "\n";
        return false;
    }
}

// Verfügbare UserSets auflisten
std::vector<std::string> getAvailableUserSets(
    const std::shared_ptr<peak::core::NodeMap> &nm)
{
    std::vector<std::string> sets;
    try {
        auto selector = nm->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector");
        for (const auto &entry : selector->Entries())
        {
            if (entry->AccessStatus() != peak::core::nodes::NodeAccessStatus::NotAvailable &&
                entry->AccessStatus() != peak::core::nodes::NodeAccessStatus::NotImplemented)
            {
                sets.push_back(entry->SymbolicValue());
            }
        }
    } catch (...) {}
    return sets;
}
```

---

## 9. Gain-Steuerung

```cpp
// GainSelector auf AnalogAll/All setzen (bevorzugt) oder DigitalAll
bool setupGainSelector(const std::shared_ptr<peak::core::NodeMap> &nm)
{
    try {
        auto selector = nm->FindNode<peak::core::nodes::EnumerationNode>("GainSelector");
        std::string chosen;

        for (const auto &entry : selector->Entries())
        {
            if (!entry->IsAvailable()) continue;
            const std::string val = entry->StringValue();
            if (val == "AnalogAll" || val == "All") {
                chosen = entry->SymbolicValue();
                break;
            }
            if (val == "DigitalAll" && chosen.empty())
                chosen = entry->SymbolicValue();
        }

        if (!chosen.empty()) {
            selector->SetCurrentEntry(chosen);
            return true;
        }
    } catch (...) {}
    return false;
}

// Gain setzen und aktuellen Wert zurückgeben
double setGain(const std::shared_ptr<peak::core::NodeMap> &nm, double value)
{
    auto gainNode = nm->FindNode<peak::core::nodes::FloatNode>("Gain");
    if (!gainNode || !gainNode->IsWriteable())
        throw std::runtime_error("Gain nicht schreibbar");

    const double clamped = std::clamp(value, gainNode->Minimum(), gainNode->Maximum());
    gainNode->SetValue(clamped);
    return gainNode->Value();
}
```

---

## 10. UserSet-Management

```cpp
// Vollständiges UserSet-Management
class UserSetManager
{
public:
    std::string current() const { return m_current; }

    std::vector<std::string> available(
        const std::shared_ptr<peak::core::NodeMap> &nm)
    {
        if (m_queried)
            return m_available;

        m_available.clear();
        try {
            auto sel = nm->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector");
            for (const auto &e : sel->Entries())
            {
                if (e->AccessStatus() != peak::core::nodes::NodeAccessStatus::NotAvailable &&
                    e->AccessStatus() != peak::core::nodes::NodeAccessStatus::NotImplemented)
                    m_available.push_back(e->SymbolicValue());
            }
        } catch (...) {}

        m_queried = true;
        return m_available;
    }

    bool load(const std::string &name, const std::shared_ptr<peak::core::NodeMap> &nm)
    {
        try {
            auto sel  = nm->FindNode<peak::core::nodes::EnumerationNode>("UserSetSelector");
            auto load = nm->FindNode<peak::core::nodes::CommandNode>("UserSetLoad");
            sel->SetCurrentEntry(name);
            load->Execute();
            load->WaitUntilDone();
            m_current = name;
            return true;
        } catch (...) { return false; }
    }

    // Für Laserscanner: welches UserSet für gegebene Belichtungszeit?
    std::string selectForExposure(float durationSec, double longExpThreshold)
    {
        bool hasLong = std::find(m_available.begin(), m_available.end(),
                                 "LongExposure") != m_available.end();
        if (hasLong && durationSec > longExpThreshold)
            return "LongExposure";
        if (std::find(m_available.begin(), m_available.end(),
                      "Default") != m_available.end())
            return "Default";
        return m_available.empty() ? "" : m_available.front();
    }

    void reset() { m_current = "Default"; m_available.clear(); m_queried = false; }

private:
    std::string m_current = "Default";
    std::vector<std::string> m_available;
    bool m_queried = false;
};
```

---

## 11. Exposure-Worker-Thread

Für nicht-blockierende Einzelbild-Aufnahmen mit Fortschrittsanzeige:

```cpp
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>

// Minimale Worker-Thread-Implementierung für Belichtungsüberwachung
class ExposureWorker
{
public:
    using TimeLeftFn   = std::function<double()>;     // gibt Restzeit in Sekunden zurück
    using ProgressFn   = std::function<void(double)>; // wird mit Restzeit aufgerufen
    using DoneFn       = std::function<void()>;       // wird bei Fertigstellung aufgerufen

    void setCallbacks(TimeLeftFn timeLeft, ProgressFn progress, DoneFn done)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_timeLeft = std::move(timeLeft);
        m_progress = std::move(progress);
        m_done     = std::move(done);
    }

    void start()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_running) return;
        m_running = true;
        m_thread  = std::thread(&ExposureWorker::run, this);
    }

    void beginExposure()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_exposureActive = true;
        m_abort = false;
        m_cv.notify_all();
    }

    void abortExposure()
    {
        { std::lock_guard<std::mutex> lock(m_mutex); m_abort = true; m_cv.notify_all(); }
        waitIdle();
    }

    void stop()
    {
        { std::lock_guard<std::mutex> lock(m_mutex); m_terminate = true; m_cv.notify_all(); }
        if (m_thread.joinable()) m_thread.join();
    }

    ~ExposureWorker() { stop(); }

private:
    void waitIdle()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_cv.wait(lock, [this]{ return !m_exposureActive; });
    }

    void run()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        while (true)
        {
            m_cv.wait(lock, [this]{ return m_exposureActive || m_terminate; });
            if (m_terminate) break;

            // Belichtungsüberwachungs-Loop
            while (m_exposureActive && !m_abort)
            {
                auto timeLeftFn  = m_timeLeft;
                auto progressFn  = m_progress;
                lock.unlock();

                double remaining = timeLeftFn ? timeLeftFn() : -1.0;
                if (progressFn) progressFn(remaining);

                lock.lock();
                if (remaining <= 0.0)
                {
                    m_exposureActive = false;
                    auto doneFn = m_done;
                    lock.unlock();
                    if (doneFn) doneFn();
                    lock.lock();
                    break;
                }

                auto waitMs = (remaining > 1.1)
                    ? std::chrono::seconds(1)
                    : std::chrono::milliseconds(100);

                m_cv.wait_for(lock, waitMs, [this]{ return m_abort || m_terminate; });
            }

            if (m_abort) {
                m_exposureActive = false;
                m_abort = false;
            }
            m_cv.notify_all();
        }
    }

    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_thread;
    bool m_running = false;
    bool m_terminate = false;
    bool m_exposureActive = false;
    bool m_abort = false;

    TimeLeftFn m_timeLeft;
    ProgressFn m_progress;
    DoneFn     m_done;
};
```

---

## 12. Pixel-Konvertierung (10/12-bit auf 16-bit)

IDS liefert 10- und 12-bit-Daten in 16-bit-Words oder gepackt. Für Weiterverarbeitung (OpenCV, FITS etc.) auf 16-bit expandieren:

```cpp
// Mono10 (16-bit-Words, Bits 9:0 genutzt) → 16-bit (MSB-ausgerichtet)
void expand10bitTo16bit(const uint8_t *src, uint8_t *dst,
                        uint32_t width, uint32_t height)
{
    const uint32_t n = width * height;
    const uint16_t *s = reinterpret_cast<const uint16_t*>(src);
    uint16_t *d = reinterpret_cast<uint16_t*>(dst);
    for (uint32_t i = 0; i < n; ++i)
        d[i] = static_cast<uint16_t>((s[i] & 0x03FF) << 6);
}

// Mono12 (16-bit-Words, Bits 11:0 genutzt) → 16-bit (MSB-ausgerichtet)
void expand12bitTo16bit(const uint8_t *src, uint8_t *dst,
                        uint32_t width, uint32_t height)
{
    const uint32_t n = width * height;
    const uint16_t *s = reinterpret_cast<const uint16_t*>(src);
    uint16_t *d = reinterpret_cast<uint16_t*>(dst);
    for (uint32_t i = 0; i < n; ++i)
        d[i] = static_cast<uint16_t>((s[i] & 0x0FFF) << 4);
}

// Mono12p (PFNC-Format: 2 Pixel in 3 Bytes) → 16-bit
// ACHTUNG: Nibble-Reihenfolge ist p0[7:0] | p0[11:8]/p1[3:0] | p1[11:4]
void expand12bitPackedTo16bit(const uint8_t *src, uint8_t *dst,
                               uint32_t width, uint32_t height)
{
    const uint32_t n = width * height;
    uint16_t *d = reinterpret_cast<uint16_t*>(dst);

    for (uint32_t i = 0; i < n; i += 2)
    {
        const uint32_t base = (i * 3) / 2;

        // Pixel 0: Byte0 + untere 4 Bits von Byte1
        const uint16_t p0 = static_cast<uint16_t>(src[base]) |
                            static_cast<uint16_t>((src[base+1] & 0x0F) << 8);
        d[i] = static_cast<uint16_t>(p0 << 4);

        // Pixel 1: obere 4 Bits von Byte1 + Byte2
        if (i + 1 < n) {
            const uint16_t p1 = static_cast<uint16_t>(src[base+1] >> 4) |
                                static_cast<uint16_t>(src[base+2] << 4);
            d[i+1] = static_cast<uint16_t>(p1 << 4);
        }
    }
}
```

---

## 13. Multi-Kamera-Betrieb

```cpp
// Jede Kamera als eigene Instanz – kein Singleton!
// Eindeutige Benennung: Modell + Seriennummer

struct CameraInstance
{
    std::string name;  // "UI-3060CP-M-GL-...-SERIAL"
    std::shared_ptr<peak::core::Device>    device;
    std::shared_ptr<peak::core::NodeMap>   nodeMap;
    std::shared_ptr<peak::core::DataStream> dataStream;
    std::mutex                              accessMutex;
};

std::vector<std::unique_ptr<CameraInstance>> openAllCameras()
{
    peak::Library::Initialize();
    auto &dm = peak::DeviceManager::Instance();
    dm.Update();

    std::vector<std::unique_ptr<CameraInstance>> cameras;

    for (const auto &desc : dm.Devices())
    {
        auto cam = std::make_unique<CameraInstance>();
        cam->name   = desc->ModelName() + "-" + desc->SerialNumber();
        cam->device = desc->OpenDevice(peak::core::DeviceAccessType::Control);
        cam->nodeMap = cam->device->RemoteDevice()->NodeMaps().at(0);
        cameras.push_back(std::move(cam));
    }

    return cameras;
}
```

---

## 14. Kritische Fallstricke und Lösungen

### Fallstrick 1: KillWait() vor StopAcquisition()

```cpp
// FALSCH: Direkt stoppen – kann hängen wenn WaitForFinishedBuffer() aktiv ist
dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);

// RICHTIG: Erst pendente Waits freigeben
dataStream->KillWait();  // Gibt WaitForFinishedBuffer() sofort zurück
dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
```

### Fallstrick 2: TLParamsLocked muss gesetzt werden

```cpp
// IDS-Empfehlung aus den SDK-Beispielen (simple_live_qtwidgets):
// TLParamsLocked auf 1 setzen VOR dataStream->StartAcquisition()
// TLParamsLocked auf 0 setzen NACH dataStream->StopAcquisition()
//
// FALSCH (ältere Methode, unzuverlässig bei uEye-Compat-Layer):
dataStream->StartAcquisition(1);  // Frame-Count-Argument
// Auf manchen Kameras liefert dies nach wenigen Frames keine Events mehr!

// RICHTIG: Unbegrenzt starten, manuell stoppen
tlParamsLockedNode->SetValue(1);
dataStream->StartAcquisition();  // Kein Argument = unbegrenzt
// ... Bild aufnehmen ...
dataStream->KillWait();
dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
tlParamsLockedNode->SetValue(0);
```

### Fallstrick 3: PayloadSize nach Format/ROI-Änderung neu lesen

```cpp
// PayloadSize ändert sich bei:
// - Format-Wechsel (z. B. Mono8 → Mono12)
// - ROI-Änderung (kleinere ROI = kleinerer Payload)
// - Binning-Änderung

// Nach jeder solchen Änderung:
// 1. Acquisition stoppen
// 2. Buffer revoken
// 3. PayloadSize neu lesen
// 4. Neue Buffer allozieren
// 5. Acquisition neu starten

void reconfigure(IDSCamera &cam)
{
    cam.stopAcquisition();
    cam.freeBuffers();
    // ... Parameter ändern ...
    auto psNode = cam.nodeMap->FindNode<peak::core::nodes::IntegerNode>("PayloadSize");
    cam.payloadSize = static_cast<size_t>(psNode->Value());
    cam.setupAcquisition();  // Neue Buffer allozieren
    cam.startAcquisition();
}
```

### Fallstrick 4: ROI-Reihenfolge beim Setzen

```cpp
// FALSCH: Direkt neue Werte setzen
widthNode->SetValue(newWidth);   // Kann Exception werfen wenn Offset+Width > Sensor!
offsetXNode->SetValue(newOffsetX);

// RICHTIG: Erst Offsets auf 0, dann Dimensionen, dann finale Werte
offsetXNode->SetValue(offsetXNode->Minimum());
offsetYNode->SetValue(offsetYNode->Minimum());
widthNode->SetValue(widthNode->Minimum());
heightNode->SetValue(heightNode->Minimum());
widthNode->SetValue(newWidth);
heightNode->SetValue(newHeight);
offsetXNode->SetValue(newOffsetX);
offsetYNode->SetValue(newOffsetY);
```

### Fallstrick 5: WaitForFinishedBuffer() wirft Exceptions bei Timeout

```cpp
// Im IDS Peak SDK ist ein Timeout KEINE normale Rückgabe – es ist eine Exception!

// FALSCH: Erwarten dass nullptr zurückgegeben wird
auto buf = dataStream->WaitForFinishedBuffer(1000);
if (!buf) { /* Timeout */ }  // BUG: Wird bei Timeout nie erreicht

// RICHTIG: Exception abfangen
try {
    auto buf = dataStream->WaitForFinishedBuffer(1000);
    // Bild verarbeiten...
} catch (const std::exception &e) {
    // Timeout oder Fehler
    std::cerr << "Timeout: " << e.what() << "\n";
}
```

### Fallstrick 6: Mono12p Nibble-Reihenfolge

```cpp
// Das PFNC-Standardformat Mono12p speichert die Nibbles SO:
//   Byte 0: p0[7:0]        (untere 8 Bits von Pixel 0)
//   Byte 1: p0[11:8] in Bits 3:0, p1[3:0] in Bits 7:4
//   Byte 2: p1[11:4]       (obere 8 Bits von Pixel 1)
//
// Die früher verwendete invertierte Reihenfolge erzeugt falsche Grauwerte!
// Korrekte Implementierung: siehe expand12bitPackedTo16bit() oben
```

---

## 15. Minimales Einzelbild-Beispiel

Vollständig lauffähiges Beispiel für einen Laserscanner-Frame:

```cpp
#include <peak/peak.hpp>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <algorithm>

int main()
{
    // 1. SDK initialisieren
    peak::Library::Initialize();

    // 2. Erste Kamera öffnen
    auto &dm = peak::DeviceManager::Instance();
    dm.Update();

    if (dm.Devices().empty())
        throw std::runtime_error("Keine IDS-Kamera gefunden");

    auto device  = dm.Devices().at(0)->OpenDevice(peak::core::DeviceAccessType::Control);
    auto nodeMap = device->RemoteDevice()->NodeMaps().at(0);

    // 3. Parameter konfigurieren
    // Format: Mono12 für maximale Dynamik (Laserlinie!)
    {
        auto fmt = nodeMap->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat");
        fmt->SetCurrentEntry("Mono12");
    }

    // Belichtungszeit: kurz für scharfe Laserlinie (z. B. 500 µs)
    {
        auto exp = nodeMap->FindNode<peak::core::nodes::FloatNode>("ExposureTime");
        double target = 500.0;
        target = std::clamp(target, exp->Minimum(), exp->Maximum());
        exp->SetValue(target);
    }

    // 4. DataStream öffnen
    auto dataStream = device->DataStreams().at(0)->OpenDataStream();

    // 5. PayloadSize und Buffer
    auto psNode = nodeMap->FindNode<peak::core::nodes::IntegerNode>("PayloadSize");
    size_t payloadSize = static_cast<size_t>(psNode->Value());

    size_t numBuffers = std::max<size_t>(
        3, static_cast<size_t>(dataStream->NumBuffersAnnouncedMinRequired()));

    for (size_t i = 0; i < numBuffers; ++i)
    {
        auto buf = dataStream->AllocAndAnnounceBuffer(payloadSize, nullptr);
        dataStream->QueueBuffer(buf);
    }

    // 6. TLParamsLocked + StartAcquisition
    try {
        auto tl = nodeMap->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked");
        if (tl->IsWriteable()) tl->SetValue(1);
    } catch (...) {}

    dataStream->StartAcquisition();

    auto startCmd = nodeMap->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart");
    startCmd->Execute();
    startCmd->WaitUntilDone();

    // 7. Bild aufnehmen
    try {
        auto buffer = dataStream->WaitForFinishedBuffer(5000);

        std::cout << "Bild empfangen: "
                  << buffer->Width() << "x" << buffer->Height()
                  << " Format=" << static_cast<int>(buffer->PixelFormat())
                  << " Bytes=" << buffer->Size() << "\n";

        // Rohdaten kopieren
        std::vector<uint8_t> raw(
            static_cast<const uint8_t*>(buffer->BasePtr()),
            static_cast<const uint8_t*>(buffer->BasePtr()) + buffer->Size());

        // TODO: Laserlinie in 'raw' suchen (Subpixel-Schwerpunkt etc.)

        dataStream->QueueBuffer(buffer);
    } catch (const std::exception &e) {
        std::cerr << "Fehler beim Bildempfang: " << e.what() << "\n";
    }

    // 8. Sauber stoppen
    try { dataStream->KillWait(); } catch (...) {}

    if (dataStream->IsGrabbing())
        dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);

    try {
        auto tl = nodeMap->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked");
        if (tl->IsWriteable()) tl->SetValue(0);
    } catch (...) {}

    try {
        auto stopCmd = nodeMap->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop");
        if (stopCmd && stopCmd->IsAvailable()) {
            stopCmd->Execute();
            stopCmd->WaitUntilDone();
        }
    } catch (...) {}

    // Buffer freigeben
    try { dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll); } catch (...) {}
    for (auto &b : dataStream->AnnouncedBuffers())
        dataStream->RevokeBuffer(b);

    peak::Library::Close();
    return 0;
}
```

---

## 16. Empfehlungen für Laserscanner-Projekte

### Kamera-Setup für optimale Laserlinien-Aufnahme

```cpp
// Typische Konfiguration für Triangulations-Laserscanner
void configureLaserScannerCamera(const std::shared_ptr<peak::core::NodeMap> &nm)
{
    // 1. Mono12 für beste Dynamik (4096 Stufen vs. 256 bei Mono8)
    //    Ermöglicht bessere Subpixel-Extraktion der Laserlinie
    auto fmt = nm->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat");
    fmt->SetCurrentEntry("Mono12");

    // 2. Kurze Belichtung (200-1000 µs) um Laserlinie scharf abzubilden
    //    und Umgebungslicht zu unterdrücken
    auto exp = nm->FindNode<peak::core::nodes::FloatNode>("ExposureTime");
    const double targetUs = 500.0;
    exp->SetValue(std::clamp(targetUs, exp->Minimum(), exp->Maximum()));

    // 3. Gain möglichst niedrig halten (weniger Rauschen = schärfere Laserlinie)
    try {
        auto gain = nm->FindNode<peak::core::nodes::FloatNode>("Gain");
        gain->SetValue(gain->Minimum());
    } catch (...) {}

    // 4. ROI auf Bereich der Laserlinie einschränken
    //    → höhere Framerate, weniger Speicher
    // setLaserLineROI(nm, ...);  // siehe Abschnitt 6
}
```

### Laserlinien-Extraktion (Schwerpunkt-Methode)

```cpp
// Subpixel-genaue Zeilenposition der Laserlinie bestimmen
// Benötigt Mono-Bild (uint8 oder uint16)
std::vector<double> extractLaserLine(
    const uint16_t *image,
    int width, int height,
    uint16_t threshold = 1000)  // Mindestschwelle für Laserreflexion
{
    std::vector<double> linePositions(width, -1.0);

    for (int x = 0; x < width; ++x)
    {
        double sumWeighted = 0.0;
        double sumWeight   = 0.0;

        for (int y = 0; y < height; ++y)
        {
            const uint16_t val = image[y * width + x];
            if (val > threshold)
            {
                sumWeighted += static_cast<double>(y) * val;
                sumWeight   += val;
            }
        }

        if (sumWeight > 0.0)
            linePositions[x] = sumWeighted / sumWeight;
        // -1.0 = kein Laserreflex in dieser Spalte
    }

    return linePositions;  // Subpixel-Zeilenposition pro Spalte
}
```

### Empfohlene Architektur für Laserscanner

```
LaserScannerApp
├── IDSCamera              (IDS Peak SDK Wrapper)
│   ├── NodeMap            (GenICam-Zugriff)
│   ├── DataStream         (Buffer-Management)
│   └── AcquisitionCtrl    (Start/Stop/Abort)
├── LaserLineExtractor     (Subpixel-Algorithmus)
├── Triangulator           (3D-Rekonstruktion)
└── PointCloudWriter       (PCD/PLY-Export)
```

### Erreichbare Frameraten (Richtwerte)

| Format | ROI         | Framerate (ca.) | Bemerkung                      |
|--------|-------------|-----------------|--------------------------------|
| Mono8  | 1280x1024   | 25 fps          | Voller Sensor                  |
| Mono8  | 1280x64     | 400+ fps        | Schmales ROI = hohe FPS        |
| Mono12 | 1280x64     | 250+ fps        | Mehr Dynamik, etwas langsamer  |
| Mono12 | 1280x32     | 500+ fps        | Für schnelle Scans             |

### Diagnosewerkzeuge

Das Projekt enthält zwei Standalone-Tools die auch im neuen Projekt nützlich sind:

```bash
# Alle Kamera-Properties auflisten
./ids_camera_inspector_start.sh

# GenICam Node-Tree der Kamera durchsuchen
./ids_iterate_nodetree_start.sh
```

Diese Tools helfen dabei, die korrekte Knoten-Namensgebung für eine spezifische Kamera zu ermitteln (manche IDS-Modelle nutzen leicht abweichende Namen).

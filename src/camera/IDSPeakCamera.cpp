#include "camera/IDSPeakCamera.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cstring>
#include <stdexcept>

IDSPeakCamera::IDSPeakCamera(const QString &deviceName, QObject *parent)
    : ICameraDevice(parent)
    , m_deviceName(deviceName)
{
    m_info.id          = QStringLiteral("ids:") + deviceName;
    m_info.displayName = deviceName;
}

IDSPeakCamera::~IDSPeakCamera()
{
    close();
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool IDSPeakCamera::open()
{
    if (m_streaming.load()) return true;

    try {
        auto &dm = peak::DeviceManager::Instance();
        dm.Update();

        std::shared_ptr<peak::core::DeviceDescriptor> found;
        const std::string target = m_deviceName.toStdString();

        for (const auto &desc : dm.Devices()) {
            if (desc->ModelName() + "-" + desc->SerialNumber() == target) {
                found = desc;
                break;
            }
        }
        if (!found)
            throw std::runtime_error("IDS camera not found: " + target);

        m_device  = found->OpenDevice(peak::core::DeviceAccessType::Control);
        m_nodeMap = m_device->RemoteDevice()->NodeMaps().at(0);

        m_width  = static_cast<int>(
            m_nodeMap->FindNode<peak::core::nodes::IntegerNode>("Width")->Maximum());
        m_height = static_cast<int>(
            m_nodeMap->FindNode<peak::core::nodes::IntegerNode>("Height")->Maximum());

        // Detect sensor type from current pixel format
        try {
            auto fmt = m_nodeMap->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat");
            const std::string cur = fmt->CurrentEntry()->SymbolicValue();
            m_isColor = (cur.find("Bayer") != std::string::npos ||
                         cur.find("RGB")   != std::string::npos ||
                         cur.find("BGR")   != std::string::npos);
        } catch (...) {}

        m_info.isColor = m_isColor;
        selectBestPixelFormat();

        // Read exposure limits
        {
            auto node = m_nodeMap->FindNode<peak::core::nodes::FloatNode>("ExposureTime");
            m_controls.exposureMin  = node->Minimum();
            m_controls.exposureMax  = node->Maximum();
            m_controls.exposureStep = node->HasConstantIncrement() ? node->Increment() : 100.0;
        }

        // Set GainSelector and read gain limits
        try {
            auto sel = m_nodeMap->FindNode<peak::core::nodes::EnumerationNode>("GainSelector");
            for (const auto &e : sel->Entries()) {
                if (!e->IsAvailable()) continue;
                const auto v = e->SymbolicValue();
                if (v == "AnalogAll" || v == "All") { sel->SetCurrentEntry(v); break; }
            }
            auto node = m_nodeMap->FindNode<peak::core::nodes::FloatNode>("Gain");
            m_controls.gainMin  = node->Minimum();
            m_controls.gainMax  = node->Maximum();
            m_controls.gainStep = 0.1;
        } catch (...) {}

        setupAcquisition();
        m_streaming = true;
        return true;

    } catch (const std::exception &e) {
        emit errorOccurred(QString::fromStdString(e.what()));
        m_nodeMap.reset();
        m_device.reset();
        return false;
    }
}

void IDSPeakCamera::close()
{
    if (!m_streaming.load()) return;
    teardownAcquisition();
    {
        std::lock_guard lock(m_nodeMutex);
        m_nodeMap.reset();
    }
    m_device.reset();
    m_streaming = false;
}

bool IDSPeakCamera::isOpen() const { return m_streaming.load(); }
CameraInfo     IDSPeakCamera::info()     const { return m_info;     }
CameraControls IDSPeakCamera::controls() const { return m_controls; }

// ---------------------------------------------------------------------------
// Format selection
// ---------------------------------------------------------------------------

void IDSPeakCamera::selectBestPixelFormat()
{
    auto fmtNode = m_nodeMap->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat");

    // Prefer formats that map cleanly to QImage without intermediate conversion.
    const std::vector<std::string> preferred = m_isColor
        ? std::vector<std::string>{"BGR8", "RGB8", "BayerRG8", "BayerGB8", "BayerGR8", "BayerBG8"}
        : std::vector<std::string>{"Mono8", "Mono10", "Mono12"};

    for (const auto &fmt : preferred) {
        try { fmtNode->SetCurrentEntry(fmt); return; } catch (...) {}
    }
}

// ---------------------------------------------------------------------------
// Acquisition pipeline
// ---------------------------------------------------------------------------

void IDSPeakCamera::setupAcquisition()
{
    m_dataStream = m_device->DataStreams().at(0)->OpenDataStream();

    const size_t payloadSize = static_cast<size_t>(
        m_nodeMap->FindNode<peak::core::nodes::IntegerNode>("PayloadSize")->Value());

    const size_t numBuffers = std::max<size_t>(
        3, static_cast<size_t>(m_dataStream->NumBuffersAnnouncedMinRequired()));

    for (size_t i = 0; i < numBuffers; ++i)
        m_dataStream->QueueBuffer(
            m_dataStream->AllocAndAnnounceBuffer(payloadSize, nullptr));

    // TLParamsLocked must be set before StartAcquisition (IDS SDK requirement)
    try {
        auto tl = m_nodeMap->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked");
        if (tl->IsWriteable()) tl->SetValue(1);
    } catch (...) {}

    m_dataStream->StartAcquisition();

    auto startCmd = m_nodeMap->FindNode<peak::core::nodes::CommandNode>("AcquisitionStart");
    startCmd->Execute();
    startCmd->WaitUntilDone();
}

void IDSPeakCamera::teardownAcquisition()
{
    // KillWait() must precede StopAcquisition() to unblock any pending WaitForFinishedBuffer
    try { m_dataStream->KillWait(); } catch (...) {}

    if (m_dataStream && m_dataStream->IsGrabbing())
        try {
            m_dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
        } catch (...) {}

    try {
        std::lock_guard lock(m_nodeMutex);
        if (m_nodeMap) {
            auto tl = m_nodeMap->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked");
            if (tl && tl->IsWriteable()) tl->SetValue(0);
            auto stopCmd = m_nodeMap->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop");
            if (stopCmd && stopCmd->IsAvailable()) {
                stopCmd->Execute();
                stopCmd->WaitUntilDone();
            }
        }
    } catch (...) {}

    if (m_dataStream) {
        try { m_dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll); } catch (...) {}
        for (auto &buf : m_dataStream->AnnouncedBuffers())
            try { m_dataStream->RevokeBuffer(buf); } catch (...) {}
        m_dataStream.reset();
    }
}

// ---------------------------------------------------------------------------
// Camera controls
// ---------------------------------------------------------------------------

bool IDSPeakCamera::setExposure(double microseconds)
{
    std::lock_guard lock(m_nodeMutex);
    if (!m_nodeMap) return false;
    try {
        auto node = m_nodeMap->FindNode<peak::core::nodes::FloatNode>("ExposureTime");
        node->SetValue(std::clamp(microseconds, node->Minimum(), node->Maximum()));
        return true;
    } catch (...) { return false; }
}

bool IDSPeakCamera::setGain(double gain)
{
    std::lock_guard lock(m_nodeMutex);
    if (!m_nodeMap) return false;
    try {
        auto node = m_nodeMap->FindNode<peak::core::nodes::FloatNode>("Gain");
        node->SetValue(std::clamp(gain, node->Minimum(), node->Maximum()));
        return true;
    } catch (...) { return false; }
}

double IDSPeakCamera::exposure() const
{
    std::lock_guard lock(m_nodeMutex);
    if (!m_nodeMap) return 0.0;
    try {
        return m_nodeMap->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->Value();
    } catch (...) { return 0.0; }
}

double IDSPeakCamera::gain() const
{
    std::lock_guard lock(m_nodeMutex);
    if (!m_nodeMap) return 0.0;
    try {
        return m_nodeMap->FindNode<peak::core::nodes::FloatNode>("Gain")->Value();
    } catch (...) { return 0.0; }
}

// ---------------------------------------------------------------------------
// Frame grab
// ---------------------------------------------------------------------------

QImage IDSPeakCamera::convertBuffer(const std::shared_ptr<peak::core::Buffer> &buffer)
{
    const int w   = static_cast<int>(buffer->Width());
    const int h   = static_cast<int>(buffer->Height());
    const auto *src = static_cast<const uint8_t*>(buffer->BasePtr());

    // Determine active pixel format
    std::string fmt;
    {
        std::lock_guard lock(m_nodeMutex);
        try {
            fmt = m_nodeMap->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat")
                            ->CurrentEntry()->SymbolicValue();
        } catch (...) { fmt = "Mono8"; }
    }

    if (fmt == "Mono8") {
        QImage img(w, h, QImage::Format_Grayscale8);
        for (int r = 0; r < h; ++r)
            std::memcpy(img.scanLine(r), src + r * w, static_cast<size_t>(w));
        return img;
    }

    if (fmt == "RGB8") {
        QImage img(w, h, QImage::Format_RGB888);
        for (int r = 0; r < h; ++r)
            std::memcpy(img.scanLine(r), src + r * w * 3, static_cast<size_t>(w * 3));
        return img;
    }

    if (fmt == "BGR8") {
        QImage img(w, h, QImage::Format_RGB888);
        for (int r = 0; r < h; ++r) {
            const uint8_t *s = src + r * w * 3;
            uint8_t *d = img.scanLine(r);
            for (int c = 0; c < w; ++c) {
                d[c * 3 + 0] = s[c * 3 + 2];
                d[c * 3 + 1] = s[c * 3 + 1];
                d[c * 3 + 2] = s[c * 3 + 0];
            }
        }
        return img;
    }

    // Bayer 8-bit formats: demosaic with OpenCV
    if (fmt.find("Bayer") != std::string::npos && fmt.find('8') != std::string::npos) {
        int cvCode = cv::COLOR_BayerRG2BGR;
        if      (fmt.find("BayerRG") != std::string::npos) cvCode = cv::COLOR_BayerRG2BGR;
        else if (fmt.find("BayerGB") != std::string::npos) cvCode = cv::COLOR_BayerGB2BGR;
        else if (fmt.find("BayerGR") != std::string::npos) cvCode = cv::COLOR_BayerGR2BGR;
        else if (fmt.find("BayerBG") != std::string::npos) cvCode = cv::COLOR_BayerBG2BGR;

        cv::Mat bayer(h, w, CV_8UC1, const_cast<uint8_t*>(src));
        cv::Mat bgr;
        cv::cvtColor(bayer, bgr, cvCode);
        QImage img(bgr.data, w, h, static_cast<int>(bgr.step), QImage::Format_BGR888);
        return img.copy(); // copy before bgr Mat goes out of scope
    }

    // Mono10 / Mono12: shift to 8-bit for display
    if (fmt == "Mono10" || fmt == "Mono12") {
        const int shift = (fmt == "Mono12") ? 4 : 2;
        const auto *s16 = reinterpret_cast<const uint16_t*>(src);
        QImage img(w, h, QImage::Format_Grayscale8);
        for (int r = 0; r < h; ++r) {
            uint8_t *d = img.scanLine(r);
            for (int c = 0; c < w; ++c)
                d[c] = static_cast<uint8_t>(s16[r * w + c] >> shift);
        }
        return img;
    }

    // Unknown format: return blank rather than crashing
    return QImage(w, h, QImage::Format_Grayscale8);
}

std::optional<QImage> IDSPeakCamera::grabFrame(int timeoutMs)
{
    if (!m_dataStream || !m_streaming.load()) return std::nullopt;

    try {
        auto buffer = m_dataStream->WaitForFinishedBuffer(
            static_cast<uint64_t>(timeoutMs));
        QImage img = convertBuffer(buffer);
        m_dataStream->QueueBuffer(buffer);
        return img;
    } catch (...) {
        return std::nullopt;
    }
}

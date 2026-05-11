#include "ui/ScannerTab.h"

#include "interfaces/ICameraDevice.h"
#include "interfaces/IPositioningStage.h"
#include "acquisition/AcquisitionThread.h"
#include "scanner/processing/LaserLineExtractor.h"
#include "scanner/processing/Triangulator.h"
#include "scanner/export/PlyWriter.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPushButton>
#include <QGroupBox>
#include <QProgressBar>
#include <QLabel>
#include <QGroupBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFile>
#include <QMetaObject>
#include <QThread>
#include <QPixmap>
#include <QPainter>
#include <QPainterPath>
#include <QSizePolicy>

#include <filesystem>
#include <span>
#include <array>
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Drain any frames that were buffered before the most recent stage move, then
// return a fresh frame captured after the stage stopped.  Without this, V4L2
// cameras may return a frame from up to ~N_buffers ago.
static std::optional<QImage> grabFreshFrame(ICameraDevice *camera, int timeoutMs)
{
    while (camera->grabFrame(0).has_value()) {}   // flush stale frames
    return camera->grabFrame(timeoutMs);
}

static scanner::Frame qImageToScannerFrame(const QImage &img)
{
    scanner::Frame frame;
    // Camera is mounted so the laser line runs vertically (top→bottom = Y axis)
    // and height (Z) displaces it horizontally (left↔right).  Rotate 90° CCW so
    // the laser becomes horizontal: columns → Y, rows → Z.  The camera preview
    // is not affected — only the scanner processing path uses this function.
    QImage gray = img.convertToFormat(QImage::Format_Grayscale8)
                     .transformed(QTransform().rotate(90));
    frame.width  = gray.width();
    frame.height = gray.height();
    frame.data.resize(static_cast<std::size_t>(gray.width() * gray.height()));
    for (int row = 0; row < gray.height(); ++row) {
        const uchar *line = gray.constScanLine(row);
        for (int col = 0; col < gray.width(); ++col)
            frame.data[static_cast<std::size_t>(row * gray.width() + col)] =
                static_cast<uint8_t>(line[col]);
    }
    return frame;
}

// ---------------------------------------------------------------------------
// Histogram helper
// ---------------------------------------------------------------------------

// threshVal is the 8-bit threshold value from the spinbox (0–255).
static QPixmap buildHistogramPixmap(const std::array<int,256> &hist, int threshVal)
{
    constexpr int    W      = 900, H = 480;
    constexpr int    mL     = 24, mR = 24, mT = 24, mB = 54;
    constexpr double kMax8 = 255.0;
    const int pW = W - mL - mR;
    const int pH = H - mT - mB;

    QPixmap pix(W, H);
    pix.fill(Qt::white);
    QPainter p(&pix);
    p.setRenderHint(QPainter::Antialiasing, false);

    // Linear scale max
    int mx = 1;
    for (int v : hist) if (v > mx) mx = v;

    // Step-function polygon — each of the 256 bins maps to one 8-bit value.
    QPainterPath path;
    path.moveTo(mL, mT + pH);
    for (int b = 0; b < 256; ++b) {
        const int bh    = static_cast<int>(static_cast<double>(hist[b]) / mx * pH);
        const double x0 = mL + b       * static_cast<double>(pW) / 256.0;
        const double x1 = mL + (b + 1) * static_cast<double>(pW) / 256.0;
        path.lineTo(x0, mT + pH - bh);
        path.lineTo(x1, mT + pH - bh);
    }
    path.lineTo(mL + pW, mT + pH);
    path.closeSubpath();

    p.setBrush(QColor(70, 110, 190, 180));
    p.setPen(QPen(QColor(40, 70, 150), 1));
    p.drawPath(path);

    // Threshold line
    if (threshVal >= 0) {
        const int tx = mL + static_cast<int>(threshVal / kMax8 * pW);
        p.setPen(QPen(Qt::red, 2));
        p.drawLine(tx, mT, tx, mT + pH);
        p.setPen(Qt::red);
        p.setFont(QFont(QString{}, 9));
        const QString lbl = QString("thresh=%1").arg(threshVal);
        const int lblX = (threshVal < 200) ? tx + 4 : tx - 95;
        p.drawText(lblX, mT + 16, lbl);
    }

    // Axes box
    p.setPen(QPen(Qt::black, 1));
    p.drawRect(mL, mT, pW, pH);

    // X ticks + labels (8-bit, 0–255)
    p.setFont(QFont(QString{}, 9));
    p.setPen(Qt::black);
    for (int v : {0, 32, 64, 96, 128, 160, 192, 224, 255}) {
        const int x = mL + static_cast<int>(v / kMax8 * pW);
        p.drawLine(x, mT + pH, x, mT + pH + 4);
        p.drawText(x - 10, mT + pH + 18, QString::number(v));
    }
    p.drawText(mL + pW / 2 - 40, H - 10, "Intensity (8-bit)");

    return pix;
}

// ---------------------------------------------------------------------------
// Construction / layout
// ---------------------------------------------------------------------------

ScannerTab::ScannerTab(QWidget *parent) : QWidget(parent)
{
    buildUI();
}

void ScannerTab::buildUI()
{
    // ── Stage control ────────────────────────────────────────────────────────
    m_stageGroup = new QGroupBox(tr("Stage"), this);
    auto *stageLay = new QVBoxLayout(m_stageGroup);

    auto *posGrid = new QGridLayout;
    posGrid->addWidget(new QLabel(tr("X:")), 0, 0);
    m_posLabelX = new QLabel(tr("—"));  posGrid->addWidget(m_posLabelX, 0, 1);
    posGrid->addWidget(new QLabel(tr("Y:")), 1, 0);
    m_posLabelY = new QLabel(tr("—"));  posGrid->addWidget(m_posLabelY, 1, 1);
    posGrid->addWidget(new QLabel(tr("Z:")), 2, 0);
    m_posLabelZ = new QLabel(tr("—"));  posGrid->addWidget(m_posLabelZ, 2, 1);
    stageLay->addLayout(posGrid);

    auto *stepRow = new QHBoxLayout;
    stepRow->addWidget(new QLabel(tr("Step:")));
    m_jogStepCombo = new QComboBox;
    for (double s : {0.001, 0.01, 0.1, 1.0, 10.0, 50.0})
        m_jogStepCombo->addItem(QStringLiteral("%1 mm").arg(s), s);
    m_jogStepCombo->setCurrentIndex(3);
    stepRow->addWidget(m_jogStepCombo);
    stageLay->addLayout(stepRow);

    auto *jogGrid = new QGridLayout;
    auto makeJog = [&](const QString &label, double dx, double dy, double dz) {
        auto *btn = new QPushButton(label, m_stageGroup);
        btn->setFixedSize(44, 28);
        connect(btn, &QPushButton::clicked, this, [this, dx, dy, dz] {
            if (!m_stage || m_scanning) return;
            const double step = m_jogStepCombo->currentData().toDouble();
            QMetaObject::invokeMethod(m_stage,
                [s = m_stage, ddx = dx*step, ddy = dy*step, ddz = dz*step] {
                    s->moveRelative(ddx, ddy, ddz);
                });
        });
        return btn;
    };
    jogGrid->addWidget(makeJog(tr("+X"),  1, 0, 0), 0, 0);
    jogGrid->addWidget(makeJog(tr("-X"), -1, 0, 0), 0, 1);
    jogGrid->addWidget(makeJog(tr("+Y"),  0, 1, 0), 1, 0);
    jogGrid->addWidget(makeJog(tr("-Y"),  0,-1, 0), 1, 1);
    jogGrid->addWidget(makeJog(tr("+Z"),  0, 0, 1), 2, 0);
    jogGrid->addWidget(makeJog(tr("-Z"),  0, 0,-1), 2, 1);
    stageLay->addLayout(jogGrid);

    // Absolute move
    stageLay->addWidget(new QLabel(tr("Go to (mm):"), m_stageGroup));
    auto makeGoSpin = [&](const QString &lbl) {
        auto *row = new QHBoxLayout;
        row->addWidget(new QLabel(lbl, m_stageGroup));
        auto *sb = new QDoubleSpinBox(m_stageGroup);
        sb->setRange(-999.999, 999.999);
        sb->setDecimals(3);
        sb->setSingleStep(0.1);
        sb->setSuffix(tr(" mm"));
        row->addWidget(sb, 1);
        stageLay->addLayout(row);
        return sb;
    };
    m_gotoX = makeGoSpin(tr("X:"));
    m_gotoY = makeGoSpin(tr("Y:"));
    m_gotoZ = makeGoSpin(tr("Z:"));

    connect(m_gotoX, &QDoubleSpinBox::valueChanged, this, [this](double) {
        if (m_gotoX->hasFocus()) m_gotoEdited = true;
    });
    connect(m_gotoY, &QDoubleSpinBox::valueChanged, this, [this](double) {
        if (m_gotoY->hasFocus()) m_gotoEdited = true;
    });
    connect(m_gotoZ, &QDoubleSpinBox::valueChanged, this, [this](double) {
        if (m_gotoZ->hasFocus()) m_gotoEdited = true;
    });

    auto *goBtn = new QPushButton(tr("Move to Position"), m_stageGroup);
    stageLay->addWidget(goBtn);
    connect(goBtn, &QPushButton::clicked, this, [this] {
        if (!m_stage || m_scanning) return;
        m_gotoEdited = false;
        const double x = m_gotoX->value(), y = m_gotoY->value(), z = m_gotoZ->value();
        QMetaObject::invokeMethod(m_stage, [s = m_stage, x, y, z] {
            s->moveAbsolute(x, y, z);
        });
    });

    auto *wcsBtn = new QPushButton(tr("Move to WCS Origin"), m_stageGroup);
    stageLay->addWidget(wcsBtn);
    connect(wcsBtn, &QPushButton::clicked, this, [this] {
        if (!m_stage || m_scanning) return;
        if (!m_stage->isHomed()) {
            QMessageBox::information(this, tr("WCS Origin"),
                tr("WCS origin not set. Use Connect → Calibrate or Set as Home first."));
            return;
        }
        QMetaObject::invokeMethod(m_stage, [s = m_stage] {
            s->moveAbsolute(0.0, 0.0, 0.0);
        });
    });

    auto *abortBtn = new QPushButton(tr("ABORT"), m_stageGroup);
    abortBtn->setStyleSheet(QStringLiteral("background-color: #cc0000; color: white;"));
    stageLay->addWidget(abortBtn);
    connect(abortBtn, &QPushButton::clicked, this, [this] {
        if (m_stage)
            QMetaObject::invokeMethod(m_stage, &IPositioningStage::abort);
    });

    // ── Scan Parameters ─────────────────────────────────────────────────────
    auto *scanGroup = new QGroupBox(tr("Scan Parameters"), this);
    auto *scanForm  = new QGridLayout(scanGroup);

    m_startXSpin = new QDoubleSpinBox; m_startXSpin->setRange(-300, 300); m_startXSpin->setDecimals(3); m_startXSpin->setSuffix(" mm");
    m_endXSpin   = new QDoubleSpinBox; m_endXSpin->setRange(-300, 300);   m_endXSpin->setDecimals(3);   m_endXSpin->setSuffix(" mm"); m_endXSpin->setValue(10.0);
    m_stepSpin   = new QDoubleSpinBox; m_stepSpin->setRange(0.001, 10); m_stepSpin->setDecimals(3); m_stepSpin->setSuffix(" mm"); m_stepSpin->setValue(0.1);

    auto *setStartBtn = new QPushButton(tr("← Pos"));
    auto *setEndBtn   = new QPushButton(tr("← Pos"));
    setStartBtn->setMaximumWidth(60);
    setEndBtn->setMaximumWidth(60);
    connect(setStartBtn, &QPushButton::clicked, this, &ScannerTab::onSetStartX);
    connect(setEndBtn,   &QPushButton::clicked, this, &ScannerTab::onSetEndX);

    scanForm->addWidget(new QLabel(tr("Start X:")), 0, 0);
    scanForm->addWidget(m_startXSpin,               0, 1);
    scanForm->addWidget(setStartBtn,                0, 2);
    scanForm->addWidget(new QLabel(tr("End X:")),   1, 0);
    scanForm->addWidget(m_endXSpin,                 1, 1);
    scanForm->addWidget(setEndBtn,                  1, 2);
    scanForm->addWidget(new QLabel(tr("Step:")),    2, 0);
    scanForm->addWidget(m_stepSpin,                 2, 1);

    // ── Camera ──────────────────────────────────────────────────────────────
    auto *camGroup = new QGroupBox(tr("Camera"), this);
    auto *camForm  = new QFormLayout(camGroup);

    m_threshSpin = new QSpinBox;
    m_threshSpin->setRange(0, 255);
    m_threshSpin->setValue(50);

    m_expSpin = new QDoubleSpinBox;
    m_expSpin->setRange(10, 100000);
    m_expSpin->setDecimals(1);
    m_expSpin->setSuffix(" µs");
    m_expSpin->setValue(500.0);

    m_scatterSpin = new QDoubleSpinBox;
    m_scatterSpin->setRange(0, 9999);
    m_scatterSpin->setDecimals(1);
    m_scatterSpin->setSuffix(" px");
    m_scatterSpin->setValue(50.0);
    m_scatterSpin->setToolTip(tr("Max deviation from median laser-line row before a column is rejected as an outlier (0 = disabled)"));

    camForm->addRow(tr("Threshold:"),    m_threshSpin);
    camForm->addRow(tr("Exposure:"),     m_expSpin);
    camForm->addRow(tr("Max scatter:"),  m_scatterSpin);

    auto *histBtn = new QPushButton(tr("Histogram…"));
    camForm->addRow(histBtn);
    connect(histBtn, &QPushButton::clicked, this, &ScannerTab::onShowHistogram);

    // ── Output file ─────────────────────────────────────────────────────────
    auto *outGroup  = new QGroupBox(tr("Output File"), this);
    auto *outLayout = new QHBoxLayout(outGroup);

    m_outputLabel = new QLabel(tr("(no file selected)"));
    m_outputLabel->setWordWrap(false);
    m_outputLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    auto *browseBtn = new QPushButton(tr("Browse…"));
    connect(browseBtn, &QPushButton::clicked, this, &ScannerTab::onBrowseOutput);
    outLayout->addWidget(m_outputLabel, 1);
    outLayout->addWidget(browseBtn);

    // ── Calibration ─────────────────────────────────────────────────────────
    auto *calibGroup = new QGroupBox(tr("Calibration"), this);
    auto *calibLayout = new QVBoxLayout(calibGroup);
    auto *calibForm  = new QFormLayout;

    m_yRefSpin   = new QDoubleSpinBox; m_yRefSpin->setRange(0, 9999);   m_yRefSpin->setDecimals(2); m_yRefSpin->setValue(512.0);
    m_scaleZSpin = new QDoubleSpinBox; m_scaleZSpin->setRange(0.0001, 10); m_scaleZSpin->setDecimals(5); m_scaleZSpin->setValue(0.05000);
    m_scaleYSpin = new QDoubleSpinBox; m_scaleYSpin->setRange(0.0001, 10); m_scaleYSpin->setDecimals(5); m_scaleYSpin->setValue(0.01700);
    m_cxSpin     = new QDoubleSpinBox; m_cxSpin->setRange(0, 9999);     m_cxSpin->setDecimals(2);   m_cxSpin->setValue(640.0);

    calibForm->addRow(tr("y_ref (px):"),   m_yRefSpin);
    calibForm->addRow(tr("scale_z (mm/px):"), m_scaleZSpin);
    calibForm->addRow(tr("scale_y (mm/px):"), m_scaleYSpin);
    calibForm->addRow(tr("cx (px):"),      m_cxSpin);
    calibLayout->addLayout(calibForm);

    auto *runCalibRow = new QHBoxLayout;
    auto *calZBtn = new QPushButton(tr("Calibrate Z…"));
    auto *calYBtn = new QPushButton(tr("Calibrate Y…"));
    connect(calZBtn, &QPushButton::clicked, this, &ScannerTab::onCalibrateZ);
    connect(calYBtn, &QPushButton::clicked, this, &ScannerTab::onCalibrateY);
    runCalibRow->addWidget(calZBtn);
    runCalibRow->addWidget(calYBtn);
    calibLayout->addLayout(runCalibRow);

    auto *calibBtnRow = new QHBoxLayout;
    auto *loadCalibBtn = new QPushButton(tr("Load JSON"));
    auto *saveCalibBtn = new QPushButton(tr("Save JSON"));
    connect(loadCalibBtn, &QPushButton::clicked, this, &ScannerTab::onLoadCalib);
    connect(saveCalibBtn, &QPushButton::clicked, this, &ScannerTab::onSaveCalib);
    calibBtnRow->addWidget(loadCalibBtn);
    calibBtnRow->addWidget(saveCalibBtn);
    calibLayout->addLayout(calibBtnRow);

    // ── Progress / status / start (Scanner panel) ────────────────────────────
    m_progress    = new QProgressBar;
    m_progress->setRange(0, 100);
    m_progress->setValue(0);
    m_statusLabel = new QLabel(tr("Idle"));

    m_startBtn = new QPushButton(tr("Start Scan"));
    m_startBtn->setMinimumHeight(36);
    connect(m_startBtn, &QPushButton::clicked, this, &ScannerTab::onStartOrAbort);

    // m_preview is not placed in any visible layout; it is kept so that
    // onFrameReady / onStepReady can still update it without crashing.
    // The persistent CameraView in MainWindow's splitter serves as the live
    // preview for all tabs.
    m_preview = new QLabel;
    m_preview->hide();

    // ── Scanner panel: control column only ───────────────────────────────────
    m_scannerPanel = new QWidget;
    auto *scannerLayout = new QVBoxLayout(m_scannerPanel);
    scannerLayout->setContentsMargins(8, 8, 8, 8);
    scannerLayout->addWidget(m_stageGroup);
    scannerLayout->addWidget(scanGroup);
    scannerLayout->addWidget(camGroup);
    scannerLayout->addWidget(outGroup);
    scannerLayout->addWidget(m_progress);
    scannerLayout->addWidget(m_statusLabel);
    scannerLayout->addWidget(m_startBtn);
    scannerLayout->addStretch();

    // ── Calibrate panel ───────────────────────────────────────────────────
    m_calibStatusLabel = new QLabel(tr("Idle"));
    m_calibStatusLabel->setWordWrap(true);
    m_calibratePanel = new QWidget;
    auto *calibPanelLayout = new QVBoxLayout(m_calibratePanel);
    calibPanelLayout->setContentsMargins(8, 8, 8, 8);
    calibPanelLayout->addWidget(calibGroup);
    calibPanelLayout->addWidget(m_calibStatusLabel);
    calibPanelLayout->addStretch();
}

// ---------------------------------------------------------------------------
// Hardware setters
// ---------------------------------------------------------------------------

void ScannerTab::setCamera(ICameraDevice *camera, AcquisitionThread *acqThread)
{
    m_camera    = camera;
    m_acqThread = acqThread;
}

void ScannerTab::setStage(IPositioningStage *stage)
{
    if (m_stage) {
        disconnect(m_stage, &IPositioningStage::movementFinished,
                   this, &ScannerTab::onStepReady);
        disconnect(m_stage, &IPositioningStage::positionChanged,
                   this, &ScannerTab::onPositionChanged);
    }
    m_stage = stage;
    if (m_stage) {
        connect(m_stage, &IPositioningStage::movementFinished,
                this, &ScannerTab::onStepReady,
                Qt::UniqueConnection);
        connect(m_stage, &IPositioningStage::positionChanged,
                this, &ScannerTab::onPositionChanged,
                Qt::UniqueConnection);
    } else {
        m_posLabelX->setText(tr("—"));
        m_posLabelY->setText(tr("—"));
        m_posLabelZ->setText(tr("—"));
    }
}

// ---------------------------------------------------------------------------
// Scan control
// ---------------------------------------------------------------------------

void ScannerTab::onStartOrAbort()
{
    if (m_scanning)
        abortScan();
    else
        startScan();
}

void ScannerTab::startScan()
{
    if (!m_camera || !m_camera->isOpen()) {
        QMessageBox::warning(this, tr("Scanner"),
            tr("Camera is not open. Start streaming in the Microscope tab first."));
        return;
    }
    if (!m_stage || !m_stage->isConnected()) {
        QMessageBox::warning(this, tr("Scanner"),
            tr("Stage is not connected."));
        return;
    }
    const double startX = m_startXSpin->value();
    const double endX   = m_endXSpin->value();
    const double step   = m_stepSpin->value();
    if (std::abs(endX - startX) < step * 0.5) {
        QMessageBox::warning(this, tr("Scanner"),
            tr("Scan range is smaller than the step size."));
        return;
    }
    const QString outPath = m_outputLabel->text();
    if (outPath.isEmpty() || outPath == tr("(no file selected)")) {
        QMessageBox::warning(this, tr("Scanner"),
            tr("Please select an output file first."));
        return;
    }

    // Build X position list — support both ascending (endX > startX) and
    // descending (endX < startX, normal after calibrate where X decreases
    // as the stage moves away from the null switch).
    const double signedStep = (endX >= startX) ? step : -step;
    m_xPositions.clear();
    for (double x = startX;
         (endX >= startX) ? (x <= endX + 1e-9) : (x >= endX - 1e-9);
         x += signedStep)
        m_xPositions.push_back(x);

    m_cloud.clear();
    m_currentStep = 0;

    // Record Y/Z from current stage position so we hold them constant
    const StagePosition pos = m_stage->position();
    m_scanY = pos.y;
    m_scanZ = pos.z;

    // Stop the live acquisition thread so the scanner can call grabFrame()
    m_acqThread->stopAcquisition();
    while (m_acqThread->isGrabbing())
        QThread::msleep(5);

    m_camera->setExposure(m_expSpin->value());

    m_scanning = true;
    m_stageGroup->setDisabled(true);
    emit scanActiveChanged(true);
    m_startBtn->setText(tr("Abort"));
    m_progress->setRange(0, static_cast<int>(m_xPositions.size()));
    m_progress->setValue(0);
    m_statusLabel->setText(tr("Moving to start…"));

    // Kick off first move; onStepReady fires when it completes
    QMetaObject::invokeMethod(m_stage,
        [s = m_stage, startX, y = m_scanY, z = m_scanZ] {
            s->moveAbsolute(startX, y, z);
        });
}

void ScannerTab::abortScan()
{
    m_scanning = false;
    m_stageGroup->setEnabled(true);
    QMetaObject::invokeMethod(m_stage, &IPositioningStage::abort);
    m_acqThread->startAcquisition();
    emit scanActiveChanged(false);
    m_startBtn->setText(tr("Start Scan"));
    m_statusLabel->setText(tr("Aborted."));
}

void ScannerTab::onStepReady(const QString &)
{
    // ── Z calibration step ───────────────────────────────────────────────────
    if (m_calibMode == CalibMode::CalibZ) {
        m_calibStatusLabel->setText(
            tr("Z-calib: capturing point %1/%2…")
                .arg(m_calibZCurrentStep + 1).arg(m_calibZTotalSteps));

        auto maybe = grabFreshFrame(m_camera, 2000);
        if (!maybe.has_value()) { finishCalibZ(false); return; }

        emit previewFrameReady(maybe.value());

        const scanner::Frame frame = qImageToScannerFrame(maybe.value());
        scanner::ExtractorParams ep;
        ep.threshold        = static_cast<uint8_t>(m_threshSpin->value());
        ep.medianRejectRows = m_scatterSpin->value();
        const double row = meanValidRow(scanner::extractLaserProfile(frame, ep));
        if (row < 0.0) {
            QMessageBox::warning(this, tr("Calibrate Z"),
                tr("No laser line at step %1 — aborting.").arg(m_calibZCurrentStep));
            finishCalibZ(false);
            return;
        }
        // Each step moves −Z, so actual Z decreases by calibZDeltaMm per step.
        const double z = m_calibZStartZ - m_calibZCurrentStep * m_calibZDeltaMm;
        m_calibZPoints.push_back({z, row});
        m_calibZCurrentStep++;

        if (m_calibZCurrentStep >= m_calibZTotalSteps) {
            finishCalibZ(true);
        } else {
            m_calibStatusLabel->setText(
                tr("Z-calib: moving to step %1/%2…")
                    .arg(m_calibZCurrentStep + 1).arg(m_calibZTotalSteps));
            QMetaObject::invokeMethod(m_stage,
                [s = m_stage, dz = m_calibZDeltaMm] { s->moveRelative(0.0, 0.0, -dz); });
        }
        return;
    }

    // ── Normal scan step ─────────────────────────────────────────────────────
    if (!m_scanning) return;

    // Capture one frame
    auto maybeFrame = grabFreshFrame(m_camera, 2000);
    if (!maybeFrame.has_value()) {
        m_scanning = false;
        m_acqThread->startAcquisition();
        emit scanActiveChanged(false);
        m_startBtn->setText(tr("Start Scan"));
        m_statusLabel->setText(tr("Error: frame capture failed."));
        return;
    }

    // Forward the grabbed frame to CameraView so the user can see what the
    // scanner captured at this step (acquisition thread is stopped during scan).
    const QImage &img = maybeFrame.value();
    emit previewFrameReady(img);

    // Convert to 8-bit scanner frame and extract laser line
    const scanner::Frame frame = qImageToScannerFrame(img);
    scanner::ExtractorParams extParams;
    extParams.threshold        = static_cast<uint8_t>(m_threshSpin->value());
    extParams.medianRejectRows = m_scatterSpin->value();
    const scanner::LaserProfile profile = scanner::extractLaserProfile(frame, extParams);

    // Project to 3D and accumulate
    const double xMm = m_xPositions[static_cast<std::size_t>(m_currentStep)];
    scanner::PointCloud stepCloud = scanner::projectTo3D(profile, xMm, currentCalib());
    m_cloud.insert(m_cloud.end(), stepCloud.begin(), stepCloud.end());

    m_currentStep++;
    m_progress->setValue(m_currentStep);
    m_statusLabel->setText(
        tr("X = %1 mm | %2 points")
            .arg(xMm, 0, 'f', 3)
            .arg(m_cloud.size()));

    if (m_currentStep >= static_cast<int>(m_xPositions.size())) {
        finishScan();
    } else {
        // Use moveRelative so the delta is independent of the stale m_position
        // in LStepStage. moveAbsolute computes (target − m_position) at call
        // time, but m_position is only updated after movementFinished fires via
        // an async enqueuePositionQuery(), so it still holds the pre-move value
        // here.  The step delta is exact from the pre-computed position list.
        const double dx = m_xPositions[static_cast<std::size_t>(m_currentStep)]
                        - m_xPositions[static_cast<std::size_t>(m_currentStep) - 1];
        QMetaObject::invokeMethod(m_stage,
            [s = m_stage, dx] {
                s->moveRelative(dx, 0.0, 0.0);
            });
    }
}

void ScannerTab::finishScan()
{
    m_scanning = false;
    m_stageGroup->setEnabled(true);
    m_acqThread->startAcquisition();
    emit scanActiveChanged(false);
    m_startBtn->setText(tr("Start Scan"));

    const QString outPath = m_outputLabel->text();
    try {
        scanner::PlyWriter writer;
        writer.write(
            std::span<const Eigen::Vector3d>(m_cloud.data(), m_cloud.size()),
            std::filesystem::path(outPath.toStdString()),
            scanner::PlyFormat::BinaryLittleEndian);
        m_statusLabel->setText(
            tr("Done. %1 points → %2").arg(m_cloud.size()).arg(outPath));
    } catch (const std::exception &e) {
        QMessageBox::warning(this, tr("Save Failed"), QString::fromStdString(e.what()));
        m_statusLabel->setText(
            tr("Done. %1 points (save failed)").arg(m_cloud.size()));
    }
}

// ---------------------------------------------------------------------------
// UI slots
// ---------------------------------------------------------------------------

void ScannerTab::onBrowseOutput()
{
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save Point Cloud"), QStringLiteral("scan.ply"),
        tr("PLY files (*.ply)"));
    if (!path.isEmpty())
        m_outputLabel->setText(path);
}

void ScannerTab::onSetStartX()
{
    if (m_stage)
        m_startXSpin->setValue(m_stage->position().x);
}

void ScannerTab::onSetEndX()
{
    if (m_stage)
        m_endXSpin->setValue(m_stage->position().x);
}

void ScannerTab::onLoadCalib()
{
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Load Calibration"), QString(), tr("JSON (*.json)"));
    if (path.isEmpty()) return;

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(this, tr("Scanner"), tr("Cannot open file."));
        return;
    }
    const QJsonObject obj = QJsonDocument::fromJson(f.readAll()).object();
    m_yRefSpin->setValue(  obj.value("y_ref")  .toDouble(m_yRefSpin->value()));
    m_scaleZSpin->setValue(obj.value("scale_z").toDouble(m_scaleZSpin->value()));
    m_scaleYSpin->setValue(obj.value("scale_y").toDouble(m_scaleYSpin->value()));
    m_cxSpin->setValue(    obj.value("cx")     .toDouble(m_cxSpin->value()));
}

void ScannerTab::onSaveCalib()
{
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save Calibration"), QStringLiteral("calib.json"),
        tr("JSON (*.json)"));
    if (path.isEmpty()) return;

    QJsonObject obj;
    obj["theta_rad"] = 0.436332;
    obj["y_ref"]   = m_yRefSpin->value();
    obj["scale_z"] = m_scaleZSpin->value();
    obj["scale_y"] = m_scaleYSpin->value();
    obj["cx"]      = m_cxSpin->value();

    QFile f(path);
    if (!f.open(QIODevice::WriteOnly))
        QMessageBox::warning(this, tr("Scanner"), tr("Cannot write file."));
    else
        f.write(QJsonDocument(obj).toJson());
}

void ScannerTab::onShowHistogram()
{
    if (!m_camera || !m_camera->isOpen()) {
        QMessageBox::warning(this, tr("Histogram"), tr("Camera is not open."));
        return;
    }

    // Factor out grab+compute so the Refresh button can reuse it.
    auto grabHist = [this]() -> std::optional<std::pair<std::array<int,256>, int>> {
        const bool was = m_acqThread && m_acqThread->isGrabbing();
        if (was) {
            m_acqThread->stopAcquisition();
            while (m_acqThread->isGrabbing()) QThread::msleep(5);
        }
        auto maybe = grabFreshFrame(m_camera, 2000);
        if (was) m_acqThread->startAcquisition();
        if (!maybe.has_value()) return std::nullopt;

        const QImage gray = maybe.value().convertToFormat(QImage::Format_Grayscale8);
        std::array<int,256> hist{};
        for (int row = 0; row < gray.height(); ++row) {
            const uchar *line = gray.constScanLine(row);
            for (int col = 0; col < gray.width(); ++col)
                ++hist[static_cast<unsigned char>(line[col])];
        }
        return std::make_pair(hist, m_threshSpin->value());
    };

    const auto result = grabHist();
    if (!result) {
        QMessageBox::warning(this, tr("Histogram"), tr("Failed to capture frame."));
        return;
    }

    auto *dlg = new QDialog(this);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowTitle(tr("Intensity Histogram"));
    auto *lay = new QVBoxLayout(dlg);

    auto *imgLabel = new QLabel(dlg);
    imgLabel->setPixmap(buildHistogramPixmap(result->first, result->second));
    lay->addWidget(imgLabel);

    auto *btnRow = new QHBoxLayout;
    auto *refreshBtn = new QPushButton(tr("Refresh"), dlg);
    auto *closeBtn   = new QPushButton(tr("Close"),   dlg);
    btnRow->addWidget(refreshBtn);
    btnRow->addStretch();
    btnRow->addWidget(closeBtn);
    lay->addLayout(btnRow);

    connect(refreshBtn, &QPushButton::clicked, dlg, [grabHist, imgLabel, dlg] {
        const auto r = grabHist();
        if (!r) return;
        imgLabel->setPixmap(buildHistogramPixmap(r->first, r->second));
        dlg->adjustSize();
    });
    connect(closeBtn, &QPushButton::clicked, dlg, &QDialog::close);

    dlg->adjustSize();
    dlg->show();
}

void ScannerTab::onFrameReady(const QImage &img)
{
    if (m_scanning || m_calibMode != CalibMode::None) return;
    m_preview->setPixmap(
        QPixmap::fromImage(img).scaled(m_preview->size(),
                                       Qt::KeepAspectRatio,
                                       Qt::FastTransformation));
}

void ScannerTab::onPositionChanged(double x, double y, double z)
{
    m_posLabelX->setText(QStringLiteral("%1 mm").arg(x, 0, 'f', 3));
    m_posLabelY->setText(QStringLiteral("%1 mm").arg(y, 0, 'f', 3));
    m_posLabelZ->setText(QStringLiteral("%1 mm").arg(z, 0, 'f', 3));
    if (!m_gotoEdited) {
        if (!m_gotoX->hasFocus() && !m_gotoY->hasFocus() && !m_gotoZ->hasFocus()) {
            { const QSignalBlocker b(m_gotoX); m_gotoX->setValue(x); }
            { const QSignalBlocker b(m_gotoY); m_gotoY->setValue(y); }
            { const QSignalBlocker b(m_gotoZ); m_gotoZ->setValue(z); }
        }
    }
}

// ---------------------------------------------------------------------------
// Calibration helpers
// ---------------------------------------------------------------------------

double ScannerTab::meanValidRow(const scanner::LaserProfile &profile)
{
    double sum = 0.0;
    int    n   = 0;
    for (double r : profile.rowPositions)
        if (r >= 0.0) { sum += r; ++n; }
    return n > 0 ? sum / n : -1.0;
}

std::pair<double,double> ScannerTab::detectEdges(const scanner::LaserProfile &profile)
{
    const auto &rows = profile.rowPositions;
    const int   sz   = static_cast<int>(rows.size());
    if (sz < 4) return {-1.0, -1.0};

    double minGrad = 0.0, maxGrad = 0.0;
    int leftIdx = -1, rightIdx = -1;

    // 5-column window: bridges gradual transitions (laser beam width) and
    // small invalid gaps (geometric shadow at the object edge).
    const int kWin = 5;
    for (int c = 0; c + kWin < sz; ++c) {
        const double r0 = rows[static_cast<std::size_t>(c)];
        const double r1 = rows[static_cast<std::size_t>(c + kWin)];
        if (r0 < 0.0 || r1 < 0.0) continue;
        const double g = r1 - r0;
        if (g < minGrad) { minGrad = g; leftIdx  = c + kWin / 2; }
        if (g > maxGrad) { maxGrad = g; rightIdx = c + kWin / 2; }
    }

    // Require at least a 5-pixel step across the window to count as an edge
    if (leftIdx < 0 || rightIdx < 0 || -minGrad < 5.0 || maxGrad < 5.0)
        return {-1.0, -1.0};

    // Return lo < hi regardless of which gradient direction ended up on which side
    const double lo = std::min(leftIdx, rightIdx) + 0.5;
    const double hi = std::max(leftIdx, rightIdx) + 0.5;
    return {lo, hi};
}

// ---------------------------------------------------------------------------
// Z calibration wizard
// ---------------------------------------------------------------------------

void ScannerTab::onCalibrateZ()
{
    if (!m_camera || !m_camera->isOpen()) {
        QMessageBox::warning(this, tr("Calibrate Z"), tr("Camera is not open."));
        return;
    }
    if (!m_stage || !m_stage->isConnected()) {
        QMessageBox::warning(this, tr("Calibrate Z"), tr("Stage is not connected."));
        return;
    }
    if (m_scanning || m_calibMode != CalibMode::None) return;

    // Ask for step size and number of steps
    QDialog dlg(this);
    dlg.setWindowTitle(tr("Z Calibration Setup"));
    auto *form = new QFormLayout(&dlg);
    auto *deltaSpin = new QDoubleSpinBox;
    deltaSpin->setRange(0.1, 10.0);
    deltaSpin->setDecimals(2);
    deltaSpin->setValue(1.0);
    deltaSpin->setSuffix(tr(" mm"));
    auto *nSpin = new QSpinBox;
    nSpin->setRange(2, 20);
    nSpin->setValue(5);
    form->addRow(tr("Step size (mm, −Z per step):"), deltaSpin);
    form->addRow(tr("Number of steps:"),              nSpin);
    form->addRow(new QLabel(tr("Place a flat, diffuse surface in the laser plane.\n"
                               "The stage will move downward (−Z) by step × n steps.\n"
                               "Make sure there is enough −Z travel from the current position.")));
    auto *btnBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btnBox, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btnBox, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    form->addRow(btnBox);
    if (dlg.exec() != QDialog::Accepted) return;

    m_calibZDeltaMm     = deltaSpin->value();
    m_calibZTotalSteps  = nSpin->value();
    m_calibZCurrentStep = 0;
    m_calibZStartZ      = m_stage->position().z;
    m_calibZPoints.clear();

    m_acqThread->stopAcquisition();
    while (m_acqThread->isGrabbing())
        QThread::msleep(5);
    m_camera->setExposure(m_expSpin->value());

    m_calibMode = CalibMode::CalibZ;
    m_stageGroup->setDisabled(true);
    m_startBtn->setDisabled(true);
    emit scanActiveChanged(true);
    m_calibStatusLabel->setText(tr("Z-calib: capturing point 1/%1…").arg(m_calibZTotalSteps));

    // Capture first frame at current Z (step 0)
    auto maybe = grabFreshFrame(m_camera, 2000);
    if (!maybe.has_value()) { finishCalibZ(false); return; }

    emit previewFrameReady(maybe.value());

    const scanner::Frame frame0 = qImageToScannerFrame(maybe.value());
    scanner::ExtractorParams ep;
    ep.threshold        = static_cast<uint8_t>(m_threshSpin->value());
    ep.medianRejectRows = m_scatterSpin->value();
    const double row0 = meanValidRow(scanner::extractLaserProfile(frame0, ep));
    if (row0 < 0.0) {
        QMessageBox::warning(this, tr("Calibrate Z"), tr("No laser line found — check threshold."));
        finishCalibZ(false);
        return;
    }
    m_calibZPoints.push_back({m_calibZStartZ, row0});
    m_calibZCurrentStep = 1;

    if (m_calibZCurrentStep >= m_calibZTotalSteps) { finishCalibZ(true); return; }

    m_calibStatusLabel->setText(tr("Z-calib: moving to step 2/%1…").arg(m_calibZTotalSteps));
    // Move downward (−Z) to stay well away from the null switch at swZ = 0.
    QMetaObject::invokeMethod(m_stage,
        [s = m_stage, dz = m_calibZDeltaMm] { s->moveRelative(0.0, 0.0, -dz); });
}

void ScannerTab::finishCalibZ(bool success)
{
    m_calibMode = CalibMode::None;
    m_stageGroup->setEnabled(true);
    m_startBtn->setEnabled(true);
    m_acqThread->startAcquisition();
    emit scanActiveChanged(false);

    // Always return to starting Z regardless of success/failure so Z is never
    // stranded partway through the calibration travel.
    if (m_stage && m_calibZCurrentStep > 0) {
        // We moved downward (−Z) m_calibZCurrentStep−1 times; return upward (+Z).
        const double returnDz = +(m_calibZCurrentStep - 1) * m_calibZDeltaMm;
        if (std::abs(returnDz) > 1e-6)
            QMetaObject::invokeMethod(m_stage,
                [s = m_stage, dz = returnDz] { s->moveRelative(0.0, 0.0, dz); });
    }

    if (!success) {
        m_calibStatusLabel->setText(tr("Z calibration failed."));
        return;
    }

    const auto calib = scanner::calibrateFromZPoints(
        std::span<const scanner::ZCalibPoint>(m_calibZPoints.data(), m_calibZPoints.size()),
        currentCalib());

    if (calib.scale_z <= 0.0) {
        QMessageBox::warning(this, tr("Calibrate Z"),
            tr("Computed scale_z is not positive — check that the laser line shifts "
               "visibly as Z changes."));
        m_calibStatusLabel->setText(tr("Z calibration failed: invalid scale_z."));
        return;
    }

    m_yRefSpin->setValue(calib.y_ref);
    m_scaleZSpin->setValue(calib.scale_z);

    m_calibStatusLabel->setText(
        tr("Z-calib done (%3 points):\ny_ref = %1 px\nscale_z = %2 mm/px")
            .arg(calib.y_ref,   0, 'f', 1)
            .arg(calib.scale_z, 0, 'f', 5)
            .arg(m_calibZPoints.size()));
}

// ---------------------------------------------------------------------------
// Y calibration wizard
// ---------------------------------------------------------------------------

void ScannerTab::onCalibrateY()
{
    if (!m_camera || !m_camera->isOpen()) {
        QMessageBox::warning(this, tr("Calibrate Y"), tr("Camera is not open."));
        return;
    }
    if (m_scanning || m_calibMode != CalibMode::None) return;

    bool ok;
    const double width = QInputDialog::getDouble(
        this, tr("Y Calibration"),
        tr("Exact width of the calibration object (mm):\n"
           "(Place the object so both edges are visible in the laser line.)"),
        10.0, 0.01, 9999.0, 3, &ok);
    if (!ok) return;

    m_acqThread->stopAcquisition();
    while (m_acqThread->isGrabbing())
        QThread::msleep(5);
    m_camera->setExposure(m_expSpin->value());
    auto maybe = grabFreshFrame(m_camera, 2000);
    m_acqThread->startAcquisition();

    if (!maybe.has_value()) {
        QMessageBox::warning(this, tr("Calibrate Y"), tr("Failed to capture frame."));
        return;
    }

    const scanner::Frame frame = qImageToScannerFrame(maybe.value());
    scanner::ExtractorParams ep;
    ep.threshold        = static_cast<uint8_t>(m_threshSpin->value());
    ep.medianRejectRows = 0.0;  // must be disabled: the object step IS the outlier we need
    const auto profile = scanner::extractLaserProfile(frame, ep);

    const auto [leftCol, rightCol] = detectEdges(profile);
    if (leftCol < 0.0 || rightCol < 0.0) {
        // Compute diagnostics to help the user understand the failure
        int    validCount   = 0;
        double rowMin       = 1e9, rowMax = -1e9, maxAdjGrad = 0.0;
        for (int c = 0; c < profile.frameWidth; ++c) {
            const double r = profile.rowPositions[static_cast<std::size_t>(c)];
            if (r >= 0.0) {
                ++validCount;
                if (r < rowMin) rowMin = r;
                if (r > rowMax) rowMax = r;
                if (c > 0) {
                    const double prev = profile.rowPositions[static_cast<std::size_t>(c - 1)];
                    if (prev >= 0.0)
                        maxAdjGrad = std::max(maxAdjGrad, std::abs(r - prev));
                }
            }
        }
        QMessageBox::warning(this, tr("Calibrate Y"),
            tr("Could not detect two edges.\n"
               "Valid columns: %1 / %2\n"
               "Row range: %3 – %4 px  (step = %5 px)\n"
               "Max adjacent gradient: %6 px\n\n"
               "Both edges must be visible with background above and below.")
                .arg(validCount).arg(profile.frameWidth)
                .arg(rowMin < 1e8 ? rowMin : 0.0, 0, 'f', 0)
                .arg(rowMax > -1e8 ? rowMax : 0.0, 0, 'f', 0)
                .arg(rowMax > -1e8 ? rowMax - rowMin : 0.0, 0, 'f', 0)
                .arg(maxAdjGrad, 0, 'f', 1));
        return;
    }

    const auto calib = scanner::calibrateFromYEdges(leftCol, rightCol, width, currentCalib());
    m_scaleYSpin->setValue(calib.scale_y);
    m_cxSpin->setValue(calib.cx);

    m_calibStatusLabel->setText(
        tr("Y-calib done: scale_y = %1 mm/px,  cx = %2 px  (edges at col %3 / %4)")
            .arg(calib.scale_y, 0, 'f', 5)
            .arg(calib.cx,      0, 'f', 1)
            .arg(leftCol,       0, 'f', 1)
            .arg(rightCol,      0, 'f', 1));
}

scanner::CalibParams ScannerTab::currentCalib() const
{
    scanner::CalibParams c;
    c.y_ref   = m_yRefSpin->value();
    c.scale_z = m_scaleZSpin->value();
    c.scale_y = m_scaleYSpin->value();
    c.cx      = m_cxSpin->value();
    return c;
}

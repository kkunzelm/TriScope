#include "ui/ScannerTab.h"

#include "interfaces/ICameraDevice.h"
#include "interfaces/IPositioningStage.h"
#include "acquisition/AcquisitionThread.h"
#include "scanner/processing/LaserLineExtractor.h"
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
#include <QFileDialog>
#include <QMessageBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFile>
#include <QMetaObject>
#include <QThread>
#include <QPixmap>
#include <QSizePolicy>

#include <filesystem>
#include <span>

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static scanner::Frame qImageToScannerFrame(const QImage &img)
{
    scanner::Frame frame;
    QImage gray = img.convertToFormat(QImage::Format_Grayscale8);
    frame.width  = gray.width();
    frame.height = gray.height();
    frame.data.resize(static_cast<std::size_t>(gray.width() * gray.height()));
    for (int row = 0; row < gray.height(); ++row) {
        const uchar *line = gray.constScanLine(row);
        for (int col = 0; col < gray.width(); ++col)
            frame.data[static_cast<std::size_t>(row * gray.width() + col)] =
                static_cast<uint16_t>(line[col]) << 8;
    }
    return frame;
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

    // ── Scan Parameters ─────────────────────────────────────────────────────
    auto *scanGroup = new QGroupBox(tr("Scan Parameters"), this);
    auto *scanForm  = new QGridLayout(scanGroup);

    m_startXSpin = new QDoubleSpinBox; m_startXSpin->setRange(0, 300); m_startXSpin->setDecimals(3); m_startXSpin->setSuffix(" mm");
    m_endXSpin   = new QDoubleSpinBox; m_endXSpin->setRange(0, 300);   m_endXSpin->setDecimals(3);   m_endXSpin->setSuffix(" mm"); m_endXSpin->setValue(10.0);
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
    m_threshSpin->setRange(0, 65535);
    m_threshSpin->setValue(500);

    m_expSpin = new QDoubleSpinBox;
    m_expSpin->setRange(10, 100000);
    m_expSpin->setDecimals(1);
    m_expSpin->setSuffix(" µs");
    m_expSpin->setValue(500.0);

    camForm->addRow(tr("Threshold:"),  m_threshSpin);
    camForm->addRow(tr("Exposure:"),   m_expSpin);

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

    auto *calibBtnRow = new QHBoxLayout;
    auto *loadCalibBtn = new QPushButton(tr("Load JSON"));
    auto *saveCalibBtn = new QPushButton(tr("Save JSON"));
    connect(loadCalibBtn, &QPushButton::clicked, this, &ScannerTab::onLoadCalib);
    connect(saveCalibBtn, &QPushButton::clicked, this, &ScannerTab::onSaveCalib);
    calibBtnRow->addWidget(loadCalibBtn);
    calibBtnRow->addWidget(saveCalibBtn);
    calibLayout->addLayout(calibBtnRow);

    // ── Progress / status ────────────────────────────────────────────────────
    m_progress    = new QProgressBar;
    m_progress->setRange(0, 100);
    m_progress->setValue(0);
    m_statusLabel = new QLabel(tr("Idle"));

    // ── Start / Abort button ─────────────────────────────────────────────────
    m_startBtn = new QPushButton(tr("Start Scan"));
    m_startBtn->setMinimumHeight(36);
    connect(m_startBtn, &QPushButton::clicked, this, &ScannerTab::onStartOrAbort);

    // ── Left panel assembly ──────────────────────────────────────────────────
    auto *leftPanel = new QWidget(this);
    leftPanel->setFixedWidth(380);
    auto *leftLayout = new QVBoxLayout(leftPanel);
    leftLayout->setContentsMargins(8, 8, 8, 8);
    leftLayout->addWidget(m_stageGroup);
    leftLayout->addWidget(scanGroup);
    leftLayout->addWidget(camGroup);
    leftLayout->addWidget(outGroup);
    leftLayout->addWidget(calibGroup);
    leftLayout->addWidget(m_progress);
    leftLayout->addWidget(m_statusLabel);
    leftLayout->addWidget(m_startBtn);
    leftLayout->addStretch();

    // ── Live preview ─────────────────────────────────────────────────────────
    m_preview = new QLabel(tr("No frame"));
    m_preview->setAlignment(Qt::AlignCenter);
    m_preview->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_preview->setMinimumSize(320, 240);
    m_preview->setStyleSheet("QLabel { background: #111; color: #888; }");

    // ── Top-level layout ─────────────────────────────────────────────────────
    auto *mainLayout = new QHBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->addWidget(leftPanel);
    mainLayout->addWidget(m_preview, 1);
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
    if (endX <= startX) {
        QMessageBox::warning(this, tr("Scanner"),
            tr("End X must be greater than Start X."));
        return;
    }
    const QString outPath = m_outputLabel->text();
    if (outPath.isEmpty() || outPath == tr("(no file selected)")) {
        QMessageBox::warning(this, tr("Scanner"),
            tr("Please select an output file first."));
        return;
    }

    // Build X position list
    m_xPositions.clear();
    for (double x = startX; x <= endX + 1e-9; x += step)
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
    if (!m_scanning) return;

    // Capture one frame
    auto maybeFrame = m_camera->grabFrame(2000);
    if (!maybeFrame.has_value()) {
        m_scanning = false;
        m_acqThread->startAcquisition();
        emit scanActiveChanged(false);
        m_startBtn->setText(tr("Start Scan"));
        m_statusLabel->setText(tr("Error: frame capture failed."));
        return;
    }

    // Update live preview
    const QImage &img = maybeFrame.value();
    m_preview->setPixmap(
        QPixmap::fromImage(img).scaled(m_preview->size(),
                                       Qt::KeepAspectRatio,
                                       Qt::FastTransformation));

    // Convert to 16-bit scanner frame and extract laser line
    const scanner::Frame frame = qImageToScannerFrame(img);
    scanner::ExtractorParams extParams;
    extParams.threshold = static_cast<uint16_t>(m_threshSpin->value());
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
        const double nextX = m_xPositions[static_cast<std::size_t>(m_currentStep)];
        QMetaObject::invokeMethod(m_stage,
            [s = m_stage, nextX, y = m_scanY, z = m_scanZ] {
                s->moveAbsolute(nextX, y, z);
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
    obj["theta_rad"] = 0.349066;
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

void ScannerTab::onPositionChanged(double x, double y, double z)
{
    m_posLabelX->setText(QStringLiteral("%1 mm").arg(x, 0, 'f', 3));
    m_posLabelY->setText(QStringLiteral("%1 mm").arg(y, 0, 'f', 3));
    m_posLabelZ->setText(QStringLiteral("%1 mm").arg(z, 0, 'f', 3));
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

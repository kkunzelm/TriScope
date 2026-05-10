#include "ui/SidebarWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QGridLayout>
#include <QScrollArea>
#include <QSerialPortInfo>
#include <QFrame>
#include <QTabWidget>

SidebarWidget::SidebarWidget(QWidget *parent)
    : QWidget(parent)
{
    auto *root = new QVBoxLayout(this);
    root->setContentsMargins(2, 2, 2, 2);
    root->setSpacing(0);

    auto *tabs = new QTabWidget(this);

    // ---- Connect tab: camera setup + stage connection/homing ----
    auto *connectPage = new QWidget;
    auto *connectLay  = new QVBoxLayout(connectPage);
    connectLay->setContentsMargins(4, 4, 4, 4);
    connectLay->setSpacing(6);
    auto *cameraGb    = new QGroupBox(tr("Camera"), connectPage);
    auto *stageConnGb = new QGroupBox(tr("Stage"),  connectPage);
    buildCameraSection(cameraGb);
    buildStageConnectSection(stageConnGb);
    connectLay->addWidget(cameraGb);
    connectLay->addWidget(stageConnGb);
    connectLay->addStretch();
    tabs->addTab(connectPage, tr("Connect"));

    // ---- Evaluate tab: position / jog / measurement tools ----
    auto *evalPage = new QWidget;
    auto *evalLay  = new QVBoxLayout(evalPage);
    evalLay->setContentsMargins(4, 4, 4, 4);
    evalLay->setSpacing(6);
    auto *stageCtrlGb = new QGroupBox(tr("Stage"),       evalPage);
    auto *overlayGb   = new QGroupBox(tr("Overlays"),    evalPage);
    auto *measureGb   = new QGroupBox(tr("Measurement"), evalPage);
    buildStageControlSection(stageCtrlGb);
    buildOverlaySection(overlayGb);
    buildMeasureSection(measureGb);
    evalLay->addWidget(stageCtrlGb);
    evalLay->addWidget(overlayGb);
    evalLay->addWidget(measureGb);
    evalLay->addStretch();
    tabs->addTab(evalPage, tr("Evaluate"));

    root->addWidget(tabs);

    setMinimumWidth(300);
    setMaximumWidth(380);
    refreshPortList();
}

// ---------------------------------------------------------------------------
// Camera section
// ---------------------------------------------------------------------------

void SidebarWidget::buildCameraSection(QGroupBox *gb)
{
    auto *lay = new QVBoxLayout(gb);

    // Camera selector
    auto *selRow = new QHBoxLayout;
    m_cameraCombo = new QComboBox(gb);
    m_cameraCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    auto *refreshBtn = new QPushButton(tr("↺"), gb);
    refreshBtn->setFixedWidth(28);
    refreshBtn->setToolTip(tr("Refresh camera list"));
    selRow->addWidget(m_cameraCombo);
    selRow->addWidget(refreshBtn);
    lay->addLayout(selRow);

    // Start / Stop
    m_streamBtn = new QPushButton(tr("Start Streaming"), gb);
    m_streamBtn->setCheckable(true);
    lay->addWidget(m_streamBtn);

    // Exposure
    lay->addWidget(new QLabel(tr("Exposure (µs):"), gb));
    auto *expRow = new QHBoxLayout;
    m_exposureSpin = new QDoubleSpinBox(gb);
    m_exposureSpin->setRange(1, 1e6);
    m_exposureSpin->setDecimals(0);
    m_exposureSpin->setSuffix(tr(" µs"));
    m_exposureSlider = new QSlider(Qt::Horizontal, gb);
    m_exposureSlider->setRange(0, 1000);
    expRow->addWidget(m_exposureSpin);
    expRow->addWidget(m_exposureSlider);
    lay->addLayout(expRow);

    // Gain
    lay->addWidget(new QLabel(tr("Gain:"), gb));
    auto *gainRow = new QHBoxLayout;
    m_gainSpin = new QDoubleSpinBox(gb);
    m_gainSpin->setRange(0, 100);
    m_gainSpin->setDecimals(2);
    m_gainSpin->setSingleStep(0.1);
    m_gainSlider = new QSlider(Qt::Horizontal, gb);
    m_gainSlider->setRange(0, 1000);
    gainRow->addWidget(m_gainSpin);
    gainRow->addWidget(m_gainSlider);
    lay->addLayout(gainRow);

    // Wiring
    connect(refreshBtn, &QPushButton::clicked, this, &SidebarWidget::cameraRefreshRequested);

    connect(m_cameraCombo, &QComboBox::currentIndexChanged,
            this, [this](int idx) {
                if (idx >= 0)
                    emit cameraSelected(m_cameraCombo->currentData().toString());
            });

    connect(m_streamBtn, &QPushButton::toggled, this, [this](bool on) {
        m_streamBtn->setText(on ? tr("Stop Streaming") : tr("Start Streaming"));
        emit cameraStartStop(on);
    });

    connect(m_exposureSpin, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        // Sync slider (avoid feedback loops)
        const QSignalBlocker bl(m_exposureSlider);
        m_exposureSlider->setValue(static_cast<int>(
            (v - m_exposureSpin->minimum()) /
            (m_exposureSpin->maximum() - m_exposureSpin->minimum()) * 1000));
        emit exposureChanged(v);
    });

    connect(m_exposureSlider, &QSlider::valueChanged, this, [this](int v) {
        const double val = m_exposureSpin->minimum() +
            v / 1000.0 * (m_exposureSpin->maximum() - m_exposureSpin->minimum());
        const QSignalBlocker bl(m_exposureSpin);
        m_exposureSpin->setValue(val);
        emit exposureChanged(val);
    });

    connect(m_gainSpin, &QDoubleSpinBox::valueChanged, this, [this](double v) {
        const QSignalBlocker bl(m_gainSlider);
        m_gainSlider->setValue(static_cast<int>(
            (v - m_gainSpin->minimum()) /
            (m_gainSpin->maximum() - m_gainSpin->minimum()) * 1000));
        emit gainChanged(v);
    });

    connect(m_gainSlider, &QSlider::valueChanged, this, [this](int v) {
        const double val = m_gainSpin->minimum() +
            v / 1000.0 * (m_gainSpin->maximum() - m_gainSpin->minimum());
        const QSignalBlocker bl(m_gainSpin);
        m_gainSpin->setValue(val);
        emit gainChanged(val);
    });
}

// ---------------------------------------------------------------------------
// Stage — Connect tab portion (port, type, connect, home, measure range)
// ---------------------------------------------------------------------------

void SidebarWidget::buildStageConnectSection(QGroupBox *gb)
{
    auto *lay = new QVBoxLayout(gb);

    auto *connRow = new QHBoxLayout;
    m_portCombo = new QComboBox(gb);
    m_stageTypeCombo = new QComboBox(gb);
    m_stageTypeCombo->addItem(tr("LStep 23"),    QStringLiteral("lstep"));
    m_stageTypeCombo->addItem(tr("DIY Stepper"), QStringLiteral("diy"));
    m_stageTypeCombo->setFixedWidth(90);
    connRow->addWidget(m_portCombo, 1);
    connRow->addWidget(m_stageTypeCombo);
    lay->addLayout(connRow);

    m_connectBtn = new QPushButton(tr("Connect"), gb);
    m_connectBtn->setCheckable(true);
    lay->addWidget(m_connectBtn);

    auto *calBtn  = new QPushButton(tr("Home (Calibrate)"), gb);
    auto *measBtn = new QPushButton(tr("Measure Range"),    gb);
    lay->addWidget(calBtn);
    lay->addWidget(measBtn);

    connect(m_connectBtn, &QPushButton::toggled, this, [this](bool on) {
        m_connectBtn->setText(on ? tr("Disconnect") : tr("Connect"));
        if (on)
            emit stageConnectRequested(m_portCombo->currentText(),
                                       m_stageTypeCombo->currentData().toString());
        else
            emit stageDisconnectRequested();
    });
    connect(calBtn,  &QPushButton::clicked, this, &SidebarWidget::calibrateRequested);
    connect(measBtn, &QPushButton::clicked, this, &SidebarWidget::measureLengthRequested);
}

// ---------------------------------------------------------------------------
// Stage — Evaluate tab portion (position, jog, goto, abort)
// ---------------------------------------------------------------------------

void SidebarWidget::buildStageControlSection(QGroupBox *gb)
{
    auto *lay = new QVBoxLayout(gb);

    // Unit toggle
    auto *unitRow = new QHBoxLayout;
    m_unitMm = new QRadioButton(tr("mm"), gb);
    m_unitUm = new QRadioButton(tr("µm"), gb);
    m_unitMm->setChecked(true);
    unitRow->addWidget(m_unitMm);
    unitRow->addWidget(m_unitUm);
    unitRow->addStretch();
    lay->addLayout(unitRow);

    // Absolute position display
    auto *posGrid = new QGridLayout;
    posGrid->addWidget(new QLabel(tr("X:"), gb), 0, 0);
    m_posLabelX = new QLabel(tr("—"), gb);
    posGrid->addWidget(m_posLabelX, 0, 1);
    posGrid->addWidget(new QLabel(tr("Y:"), gb), 1, 0);
    m_posLabelY = new QLabel(tr("—"), gb);
    posGrid->addWidget(m_posLabelY, 1, 1);
    posGrid->addWidget(new QLabel(tr("Z:"), gb), 2, 0);
    m_posLabelZ = new QLabel(tr("—"), gb);
    posGrid->addWidget(m_posLabelZ, 2, 1);
    lay->addLayout(posGrid);

    // Table measurement: set origin, show relative XY
    m_setOriginBtn = new QPushButton(tr("Set Origin (Zero ΔX/ΔY)"), gb);
    lay->addWidget(m_setOriginBtn);

    auto *relGrid = new QGridLayout;
    relGrid->addWidget(new QLabel(tr("ΔX:"), gb), 0, 0);
    m_relLabelX = new QLabel(tr("—"), gb);
    relGrid->addWidget(m_relLabelX, 0, 1);
    relGrid->addWidget(new QLabel(tr("ΔY:"), gb), 1, 0);
    m_relLabelY = new QLabel(tr("—"), gb);
    relGrid->addWidget(m_relLabelY, 1, 1);
    lay->addLayout(relGrid);

    connect(m_setOriginBtn, &QPushButton::clicked, this, [this] {
        m_originX   = m_absX;
        m_originY   = m_absY;
        m_originSet = true;
        m_relLabelX->setText(formatPosition(0.0));
        m_relLabelY->setText(formatPosition(0.0));
    });

    // Jog step selector
    auto *stepRow = new QHBoxLayout;
    stepRow->addWidget(new QLabel(tr("Step:"), gb));
    m_stepCombo = new QComboBox(gb);
    for (double s : {0.001, 0.01, 0.1, 1.0, 10.0, 50.0})
        m_stepCombo->addItem(QStringLiteral("%1 mm").arg(s), s);
    m_stepCombo->setCurrentIndex(3);
    stepRow->addWidget(m_stepCombo);
    lay->addLayout(stepRow);

    // Jog buttons (3×2 grid: X/Y/Z ±)
    auto *jogGrid = new QGridLayout;
    auto makeJog = [&](const QString &label, double dx, double dy, double dz) {
        auto *btn = new QPushButton(label, gb);
        btn->setFixedSize(44, 28);
        connect(btn, &QPushButton::clicked, this, [this, dx, dy, dz] {
            emit jogRequested(dx * jogStep(), dy * jogStep(), dz * jogStep());
        });
        return btn;
    };
    jogGrid->addWidget(makeJog(tr("+X"),  1, 0, 0), 0, 0);
    jogGrid->addWidget(makeJog(tr("-X"), -1, 0, 0), 0, 1);
    jogGrid->addWidget(makeJog(tr("+Y"),  0, 1, 0), 1, 0);
    jogGrid->addWidget(makeJog(tr("-Y"),  0,-1, 0), 1, 1);
    jogGrid->addWidget(makeJog(tr("+Z"),  0, 0, 1), 2, 0);
    jogGrid->addWidget(makeJog(tr("-Z"),  0, 0,-1), 2, 1);
    lay->addLayout(jogGrid);

    // Absolute move
    lay->addWidget(new QLabel(tr("Go to (mm):"), gb));
    auto makeGoSpin = [&](const QString &label) {
        auto *row = new QHBoxLayout;
        row->addWidget(new QLabel(label, gb));
        auto *sb = new QDoubleSpinBox(gb);
        sb->setRange(-999.999, 999.999);
        sb->setDecimals(3);
        sb->setSingleStep(0.1);
        sb->setSuffix(tr(" mm"));
        row->addWidget(sb, 1);
        lay->addLayout(row);
        return sb;
    };
    m_gotoX = makeGoSpin(tr("X:"));
    m_gotoY = makeGoSpin(tr("Y:"));
    m_gotoZ = makeGoSpin(tr("Z:"));

    auto *goBtn = new QPushButton(tr("Move to Position"), gb);
    lay->addWidget(goBtn);
    connect(goBtn, &QPushButton::clicked, this, [this] {
        emit moveAbsoluteRequested(m_gotoX->value(), m_gotoY->value(), m_gotoZ->value());
    });

    // ABORT
    auto *abortBtn = new QPushButton(tr("ABORT"), gb);
    abortBtn->setStyleSheet(QStringLiteral("background-color: #cc0000; color: white;"));
    lay->addWidget(abortBtn);

    auto *unitGroup = new QButtonGroup(gb);
    unitGroup->addButton(m_unitMm, 0);
    unitGroup->addButton(m_unitUm, 1);
    connect(unitGroup, &QButtonGroup::idClicked, this, [this](int id) {
        m_showUm = (id == 1);
        emit unitToggled(m_showUm);
    });
    connect(abortBtn, &QPushButton::clicked, this, &SidebarWidget::abortRequested);
}

// ---------------------------------------------------------------------------
// Overlay section
// ---------------------------------------------------------------------------

void SidebarWidget::buildOverlaySection(QGroupBox *gb)
{
    auto *lay = new QVBoxLayout(gb);

    auto *xhair = new QCheckBox(tr("Crosshair"), gb);
    auto *grid  = new QCheckBox(tr("10×10 Grid"), gb);
    lay->addWidget(xhair);
    lay->addWidget(grid);

    connect(xhair, &QCheckBox::toggled, this, &SidebarWidget::crosshairToggled);
    connect(grid,  &QCheckBox::toggled, this, &SidebarWidget::gridToggled);
}

// ---------------------------------------------------------------------------
// Measurement section
// ---------------------------------------------------------------------------

void SidebarWidget::buildMeasureSection(QGroupBox *gb)
{
    auto *lay = new QVBoxLayout(gb);

    // 2×2 grid so "Distance" isn't clipped in a too-narrow single row
    auto *distBtn  = new QPushButton(tr("Distance"), gb);
    auto *angleBtn = new QPushButton(tr("Angle"),    gb);
    auto *radBtn   = new QPushButton(tr("Radius"),   gb);
    auto *clrBtn   = new QPushButton(tr("Clear"),    gb);
    auto *toolGrid = new QGridLayout;
    toolGrid->addWidget(distBtn,  0, 0);
    toolGrid->addWidget(angleBtn, 0, 1);
    toolGrid->addWidget(radBtn,   1, 0);
    toolGrid->addWidget(clrBtn,   1, 1);
    lay->addLayout(toolGrid);

    // Calibration row
    lay->addWidget(new QLabel(tr("Calibration:"), gb));
    auto *calRow = new QHBoxLayout;
    m_calPixSpin = new QDoubleSpinBox(gb);
    m_calPixSpin->setRange(1, 100000);
    m_calPixSpin->setDecimals(1);
    m_calPixSpin->setSuffix(tr(" px"));
    m_calPixSpin->setValue(100);
    m_calUmSpin = new QDoubleSpinBox(gb);
    m_calUmSpin->setRange(0.1, 1e7);
    m_calUmSpin->setDecimals(1);
    m_calUmSpin->setSuffix(tr(" µm"));
    m_calUmSpin->setValue(100);
    auto *setCalBtn = new QPushButton(tr("Set"), gb);
    setCalBtn->setFixedWidth(36);
    calRow->addWidget(m_calPixSpin);
    calRow->addWidget(new QLabel(tr("="), gb));
    calRow->addWidget(m_calUmSpin);
    calRow->addWidget(setCalBtn);
    lay->addLayout(calRow);

    // Wiring
    connect(distBtn,  &QPushButton::clicked, this, [this] {
        emit measurementToolSelected(MeasurementOverlay::Mode::Distance); });
    connect(angleBtn, &QPushButton::clicked, this, [this] {
        emit measurementToolSelected(MeasurementOverlay::Mode::Angle); });
    connect(radBtn,   &QPushButton::clicked, this, [this] {
        emit measurementToolSelected(MeasurementOverlay::Mode::Radius); });
    connect(clrBtn,   &QPushButton::clicked, this, &SidebarWidget::measurementCleared);
    connect(setCalBtn, &QPushButton::clicked, this, [this] {
        emit calibrationSet(m_calPixSpin->value(), m_calUmSpin->value());
    });
}

// ---------------------------------------------------------------------------
// Public update methods
// ---------------------------------------------------------------------------

void SidebarWidget::setCameraList(const QList<CameraInfo> &cameras)
{
    const QSignalBlocker bl(m_cameraCombo);
    m_cameraCombo->clear();
    for (const auto &c : cameras)
        m_cameraCombo->addItem(c.displayName, c.id);
}

void SidebarWidget::updateCameraControls(const CameraControls &ctrl,
                                          double currentExposure, double currentGain)
{
    {
        const QSignalBlocker bl1(m_exposureSpin), bl2(m_exposureSlider);
        m_exposureSpin->setRange(ctrl.exposureMin, ctrl.exposureMax);
        m_exposureSpin->setSingleStep(ctrl.exposureStep);
        m_exposureSpin->setValue(currentExposure);
    }
    {
        const QSignalBlocker bl1(m_gainSpin), bl2(m_gainSlider);
        m_gainSpin->setRange(ctrl.gainMin, ctrl.gainMax);
        m_gainSpin->setSingleStep(ctrl.gainStep);
        m_gainSpin->setValue(currentGain);
    }
}

void SidebarWidget::updatePosition(double x, double y, double z)
{
    m_absX = x;
    m_absY = y;
    m_posLabelX->setText(formatPosition(x));
    m_posLabelY->setText(formatPosition(y));
    m_posLabelZ->setText(formatPosition(z));
    if (m_originSet) {
        m_relLabelX->setText(formatPosition(x - m_originX));
        m_relLabelY->setText(formatPosition(y - m_originY));
    }
}

void SidebarWidget::refreshPortList()
{
    const QSignalBlocker bl(m_portCombo);
    const QString current = m_portCombo->currentText();
    m_portCombo->clear();
    for (const auto &info : QSerialPortInfo::availablePorts())
        m_portCombo->addItem(info.portName());
    if (!current.isEmpty()) {
        const int idx = m_portCombo->findText(current);
        if (idx >= 0) m_portCombo->setCurrentIndex(idx);
    }
}

double SidebarWidget::jogStep() const
{
    return m_stepCombo->currentData().toDouble();
}

QString SidebarWidget::formatPosition(double mm) const
{
    if (m_showUm)
        return QStringLiteral("%1 µm").arg(mm * 1000.0, 0, 'f', 1);
    return QStringLiteral("%1 mm").arg(mm, 0, 'f', 3);
}

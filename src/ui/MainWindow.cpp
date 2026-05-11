#include "ui/MainWindow.h"

#include "ui/CameraView.h"
#include "ui/SidebarWidget.h"
#include "ui/ScannerTab.h"
#include "camera/CameraDiscovery.h"
#include "camera/IDSPeakCamera.h"
#include "camera/V4L2Camera.h"
#include "acquisition/AcquisitionThread.h"
#include "stage/LStepStage.h"
#include "stage/DIYStepperStage.h"

#include <QHBoxLayout>
#include <QSplitter>
#include <QScrollArea>
#include <QTabWidget>
#include <QStatusBar>
#include <QMenuBar>
#include <QMenu>
#include <QApplication>
#include <QScreen>
#include <QThread>
#include <QMessageBox>
#include <QLabel>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle(tr("Video Measuring Microscope"));
    // Size to ~90 % of available screen so all sidebar items are visible without manual resizing
    if (const auto *screen = QApplication::primaryScreen()) {
        const QRect avail = screen->availableGeometry();
        resize(qMin(1500, avail.width()  - 80),
               qMin(1000, avail.height() - 60));
        move(avail.topLeft() + QPoint(40, 30));
    } else {
        resize(1280, 900);
    }

    // ---- Discovery ----
    m_discovery = new CameraDiscovery(this);

    // ---- Controllers (not directly visible) ----
    m_sidebar    = new SidebarWidget(this);
    m_scannerTab = new ScannerTab(this);
    m_cameraView = new CameraView;

    // ---- Tab 1: Connect — Camera + Stage setup ----
    auto *connectScroll = new QScrollArea;
    connectScroll->setWidget(m_sidebar->connectPanel());
    connectScroll->setWidgetResizable(true);
    connectScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    connectScroll->setMaximumWidth(420);

    // ---- Tab 2: Microscope — jog/measure sidebar only (no camera here) ----
    auto *evalScroll = new QScrollArea;
    evalScroll->setWidget(m_sidebar->evaluatePanel());
    evalScroll->setWidgetResizable(true);
    evalScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    // ---- Tab 3: Scanner — stage jog + scan controls ----
    // (m_scannerTab->scannerPanel() is a plain VBox of controls)

    // ---- Tab 4: Calibrate — calibration wizards + parameters ----
    auto *calibScroll = new QScrollArea;
    calibScroll->setWidget(m_scannerTab->calibratePanel());
    calibScroll->setWidgetResizable(true);
    calibScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    // ---- Central tab widget (left panel, max 420 px) ----
    m_tabs = new QTabWidget;
    m_tabs->addTab(connectScroll,                tr("Connect"));
    m_tabs->addTab(evalScroll,                   tr("Microscope"));
    m_tabs->addTab(m_scannerTab->scannerPanel(), tr("Scanner"));
    m_tabs->addTab(calibScroll,                  tr("Calibrate"));
    m_tabs->setMaximumWidth(420);

    // ---- Persistent central splitter: tabs on left, camera always on right ----
    auto *centralSplitter = new QSplitter(Qt::Horizontal, this);
    centralSplitter->addWidget(m_tabs);
    centralSplitter->addWidget(m_cameraView);
    centralSplitter->setStretchFactor(0, 0);
    centralSplitter->setStretchFactor(1, 1);
    setCentralWidget(centralSplitter);

    // ---- Acquisition thread ----
    m_acqThread = new AcquisitionThread(this);
    connect(m_acqThread, &AcquisitionThread::frameReady,
            m_cameraView, &CameraView::onFrameReady,
            Qt::QueuedConnection);
    connect(m_acqThread, &AcquisitionThread::frameReady,
            m_scannerTab, &ScannerTab::onFrameReady,
            Qt::QueuedConnection);
    connect(m_acqThread, &AcquisitionThread::acquisitionError,
            this, &MainWindow::onAcquisitionError);

    // ---- Stage thread ----
    m_stageThread = new QThread(this);
    m_stageThread->setObjectName(QStringLiteral("StageThread"));
    m_stageThread->start();

    // ---- Menu ----
    buildMenu();

    // ---- Sidebar → MainWindow wiring ----
    connect(m_sidebar, &SidebarWidget::cameraRefreshRequested,
            this, &MainWindow::onCameraRefresh);
    connect(m_sidebar, &SidebarWidget::cameraSelected,
            this, &MainWindow::onCameraSelected);
    connect(m_sidebar, &SidebarWidget::cameraStartStop,
            this, &MainWindow::onCameraStartStop);
    connect(m_sidebar, &SidebarWidget::exposureChanged,
            this, &MainWindow::onExposureChanged);
    connect(m_sidebar, &SidebarWidget::gainChanged,
            this, &MainWindow::onGainChanged);
    connect(m_sidebar, &SidebarWidget::stageConnectRequested,
            this, &MainWindow::onStageConnect);
    connect(m_sidebar, &SidebarWidget::stageDisconnectRequested,
            this, &MainWindow::onStageDisconnect);
    connect(m_sidebar, &SidebarWidget::jogRequested,
            this, &MainWindow::onJog);
    connect(m_sidebar, &SidebarWidget::moveAbsoluteRequested,
            this, &MainWindow::onMoveAbsolute);
    connect(m_sidebar, &SidebarWidget::calibrateRequested,
            this, &MainWindow::onCalibrate);
    connect(m_sidebar, &SidebarWidget::setHomeRequested,
            this, &MainWindow::onSetHome);
    connect(m_sidebar, &SidebarWidget::measureLengthRequested,
            this, &MainWindow::onMeasureLength);
    connect(m_sidebar, &SidebarWidget::abortRequested,
            this, &MainWindow::onAbort);
    connect(m_sidebar, &SidebarWidget::crosshairToggled,
            m_cameraView, &CameraView::setCrosshairVisible);
    connect(m_sidebar, &SidebarWidget::gridToggled,
            m_cameraView, &CameraView::setGridVisible);
    connect(m_sidebar, &SidebarWidget::measurementToolSelected,
            m_cameraView, &CameraView::startMeasurement);
    connect(m_sidebar, &SidebarWidget::measurementCleared,
            m_cameraView, &CameraView::clearMeasurements);
    connect(m_sidebar, &SidebarWidget::calibrationSet,
            this, [this](double pixels, double um) {
                m_cameraView->setScale(um / pixels);
            });
    connect(m_cameraView, &CameraView::measurementResult,
            this, [this](const QString &txt) { statusBar()->showMessage(txt, 5000); });

    // Disable Microscope jog panel while a scan is running
    connect(m_scannerTab, &ScannerTab::scanActiveChanged,
            m_sidebar->evaluatePanel(), &QWidget::setDisabled);

    // Initial camera list
    onCameraRefresh();

    statusBar()->showMessage(tr("Ready"));
}

MainWindow::~MainWindow()
{
    m_acqThread->stopAcquisition();
    m_acqThread->requestInterruption();
    m_acqThread->wait(2000);

    if (m_camera) {
        m_camera->close();
        m_camera.reset();
    }

    disconnectStage();

    m_stageThread->quit();
    m_stageThread->wait(5000);
}

// ---------------------------------------------------------------------------
// Menu
// ---------------------------------------------------------------------------

void MainWindow::buildMenu()
{
    auto *fileMenu = menuBar()->addMenu(tr("&File"));
    auto *quitAct = fileMenu->addAction(tr("&Quit"), this, &QMainWindow::close);
    quitAct->setShortcut(QKeySequence::Quit);

    auto *viewMenu = menuBar()->addMenu(tr("&View"));
    auto *fullscreenAct = viewMenu->addAction(tr("&Fullscreen"));
    fullscreenAct->setCheckable(true);
    fullscreenAct->setShortcut(QKeySequence::FullScreen);
    connect(fullscreenAct, &QAction::toggled, this, [this](bool on) {
        on ? showFullScreen() : showNormal();
    });

    auto *helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(tr("About Qt"), qApp, &QApplication::aboutQt);
}

// ---------------------------------------------------------------------------
// Camera slots
// ---------------------------------------------------------------------------

void MainWindow::onCameraRefresh()
{
    const auto cameras = m_discovery->discover();
    m_sidebar->setCameraList(cameras);
    // Auto-select the first camera so m_camera is valid before the user
    // clicks Start — setCameraList blocks currentIndexChanged signals.
    if (!cameras.isEmpty())
        onCameraSelected(cameras.first().id);
    statusBar()->showMessage(
        tr("Found %1 camera(s)").arg(cameras.size()), 3000);
}

void MainWindow::onCameraSelected(const QString &id)
{
    // Stop existing stream before switching cameras
    if (m_camera && m_camera->isOpen()) {
        m_acqThread->stopAcquisition();
        m_camera->close();
    }
    m_camera = m_discovery->createCamera(id);
    m_scannerTab->setCamera(m_camera.get(), m_acqThread);
    statusBar()->showMessage(tr("Camera selected: %1").arg(id), 2000);
}

void MainWindow::onCameraStartStop(bool start)
{
    if (!m_camera) {
        statusBar()->showMessage(tr("No camera selected"), 2000);
        return;
    }

    if (start) {
        if (!m_camera->open()) {
            statusBar()->showMessage(tr("Failed to open camera"), 3000);
            return;
        }
        m_sidebar->updateCameraControls(m_camera->controls(),
                                        m_camera->exposure(),
                                        m_camera->gain());
        m_acqThread->setCamera(m_camera.get());
        m_acqThread->startAcquisition();
        statusBar()->showMessage(tr("Streaming started"), 2000);
    } else {
        m_acqThread->stopAcquisition();
        m_camera->close();
        statusBar()->showMessage(tr("Streaming stopped"), 2000);
    }
}

void MainWindow::onExposureChanged(double us)
{
    if (m_camera) m_camera->setExposure(us);
}

void MainWindow::onGainChanged(double gain)
{
    if (m_camera) m_camera->setGain(gain);
}

void MainWindow::onAcquisitionError(const QString &msg)
{
    statusBar()->showMessage(tr("Acquisition error: %1").arg(msg), 5000);
}

// ---------------------------------------------------------------------------
// Stage slots
// ---------------------------------------------------------------------------

void MainWindow::onStageConnect(const QString &port, const QString &type)
{
    disconnectStage();

    IPositioningStage *stage = nullptr;
    if (type == QStringLiteral("lstep"))
        stage = new LStepStage;
    else
        stage = new DIYStepperStage;

    stage->moveToThread(m_stageThread);
    m_stage.reset(stage);

    connectStageSignals(stage);
    m_scannerTab->setStage(stage);

    // Connect is called in the stage's thread
    QMetaObject::invokeMethod(stage, [stage, port] { stage->connect(port); });
}

void MainWindow::onStageDisconnect()
{
    disconnectStage();
}

void MainWindow::connectStageSignals(IPositioningStage *stage)
{
    connect(stage, &IPositioningStage::positionChanged,
            this, &MainWindow::onPositionChanged);
    connect(stage, &IPositioningStage::movementFinished,
            this, &MainWindow::onMovementFinished);
    connect(stage, &IPositioningStage::errorOccurred,
            this, &MainWindow::onStageError);
    connect(stage, &IPositioningStage::connected, this, [this] {
        statusBar()->showMessage(tr("Stage connected"), 2000);
    });
    connect(stage, &IPositioningStage::disconnected, this, [this] {
        statusBar()->showMessage(tr("Stage disconnected"), 2000);
    });
}

void MainWindow::disconnectStage()
{
    if (!m_stage) return;
    m_scannerTab->setStage(nullptr);
    // BlockingQueuedConnection: main thread waits until disconnect() returns in
    // the stage thread. If the stage thread is mid-command (waitForBytesWritten),
    // we wait up to ~2.5 s for it to finish before the abort+close runs.
    QMetaObject::invokeMethod(m_stage.get(), &IPositioningStage::disconnect,
                              Qt::BlockingQueuedConnection);
    m_stage.reset();
}

void MainWindow::onJog(double dx, double dy, double dz)
{
    if (m_stage)
        QMetaObject::invokeMethod(m_stage.get(), [s = m_stage.get(), dx, dy, dz] {
            s->moveRelative(dx, dy, dz);
        });
}

void MainWindow::onMoveAbsolute(double x, double y, double z)
{
    if (m_stage)
        QMetaObject::invokeMethod(m_stage.get(), [s = m_stage.get(), x, y, z] {
            s->moveAbsolute(x, y, z);
        });
}

void MainWindow::onCalibrate()
{
    if (m_stage)
        QMetaObject::invokeMethod(m_stage.get(), &IPositioningStage::calibrate);
}

void MainWindow::onSetHome()
{
    if (m_stage)
        QMetaObject::invokeMethod(m_stage.get(), &IPositioningStage::setHome);
}

void MainWindow::onMeasureLength()
{
    if (auto *ls = qobject_cast<LStepStage*>(m_stage.get()))
        QMetaObject::invokeMethod(ls, &LStepStage::measureLength);
}

void MainWindow::onAbort()
{
    if (m_stage)
        QMetaObject::invokeMethod(m_stage.get(), &IPositioningStage::abort);
}

void MainWindow::onPositionChanged(double x, double y, double z)
{
    m_sidebar->updatePosition(x, y, z);
}

void MainWindow::onMovementFinished(const QString &status)
{
    statusBar()->showMessage(tr("Move done: %1").arg(status), 2000);
}

void MainWindow::onStageError(const QString &msg)
{
    statusBar()->showMessage(tr("Stage: %1").arg(msg), 5000);
}

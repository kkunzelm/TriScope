#include "ui/MainWindow.h"

#include "ui/CameraView.h"
#include "ui/SidebarWidget.h"
#include "camera/CameraDiscovery.h"
#include "camera/IDSPeakCamera.h"
#include "camera/V4L2Camera.h"
#include "acquisition/AcquisitionThread.h"
#include "stage/LStepStage.h"
#include "stage/DIYStepperStage.h"

#include <QHBoxLayout>
#include <QSplitter>
#include <QScrollArea>
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

    // ---- Central layout: sidebar | camera view ----
    auto *splitter = new QSplitter(Qt::Horizontal, this);

    m_sidebar = new SidebarWidget(this);
    auto *scrollArea = new QScrollArea(this);
    scrollArea->setWidget(m_sidebar);
    scrollArea->setWidgetResizable(true);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setMaximumWidth(400);
    splitter->addWidget(scrollArea);

    m_cameraView = new CameraView(this);
    splitter->addWidget(m_cameraView);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);

    setCentralWidget(splitter);

    // ---- Acquisition thread ----
    m_acqThread = new AcquisitionThread(this);
    connect(m_acqThread, &AcquisitionThread::frameReady,
            m_cameraView, &CameraView::onFrameReady,
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
    m_stageThread->wait(2000);
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
    if (m_stage) {
        QMetaObject::invokeMethod(m_stage.get(), &IPositioningStage::disconnect);
        // Give the stage thread a moment to process disconnect
        QThread::msleep(100);
        m_stage.reset();
    }
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

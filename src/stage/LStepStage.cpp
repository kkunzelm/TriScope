#include "stage/LStepStage.h"

#include <cmath>

// ---------------------------------------------------------------------------
// MCL3 protocol helpers
// ---------------------------------------------------------------------------

QByteArray LStepStage::mcl3(uint8_t reg, const QString &value)
{
    QByteArray cmd;
    cmd.append(static_cast<char>(0x55));
    cmd.append(static_cast<char>(reg));
    if (!value.isEmpty())
        cmd.append(value.toLatin1());
    cmd.append(static_cast<char>(0x0D));
    return cmd;
}

QByteArray LStepStage::upCmd()
{
    // 0x55 0x50 0x0D  ("UP" in the compact reference)
    return QByteArray("\x55\x50\x0D", 3);
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

LStepStage::LStepStage(QObject *parent)
    : IPositioningStage(parent)
    , m_port(new QSerialPort(this))
    , m_cmdTimeout(new QTimer(this))
    , m_pollTimer(new QTimer(this))
{
    m_cmdTimeout->setSingleShot(true);
    QObject::connect(m_cmdTimeout, &QTimer::timeout,
                     this, &LStepStage::onCommandTimeout);

    m_pollTimer->setInterval(500);
    QObject::connect(m_pollTimer, &QTimer::timeout,
                     this, &LStepStage::onPollTimer);
}

LStepStage::~LStepStage()
{
    disconnect();
}

// ---------------------------------------------------------------------------
// Connection
// ---------------------------------------------------------------------------

bool LStepStage::connect(const QString &portName)
{
    if (m_connected) return true;

    m_port->setPortName(portName);
    m_port->setBaudRate(QSerialPort::Baud19200);
    m_port->setDataBits(QSerialPort::Data8);
    m_port->setParity(QSerialPort::NoParity);
    m_port->setStopBits(QSerialPort::TwoStop);          // critical: 2 stop bits

    // Pseudo-terminals (socat) have no real RTS/CTS lines → use NoFlowControl
    const bool isPty = portName.startsWith(QStringLiteral("/tmp/")) ||
                       portName.startsWith(QStringLiteral("/dev/pts/"));
    m_hwFlowControl = !isPty;
    m_port->setFlowControl(m_hwFlowControl ? QSerialPort::HardwareControl
                                           : QSerialPort::NoFlowControl);

    if (!m_port->open(QIODevice::ReadWrite)) {
        emit errorOccurred(QStringLiteral("Cannot open %1: %2")
                           .arg(portName, m_port->errorString()));
        return false;
    }

    m_port->setDataTerminalReady(true);
    m_port->setRequestToSend(true);
    m_port->clear(QSerialPort::AllDirections);

    QObject::connect(m_port, &QSerialPort::readyRead,
                     this, &LStepStage::onReadyRead);

    m_connected = true;
    sendInitSequence();
    m_pollTimer->start();

    emit connected();
    return true;
}

void LStepStage::disconnect()
{
    if (!m_connected) return;
    m_pollTimer->stop();
    m_cmdTimeout->stop();
    m_queue.clear();
    m_busy = false;

    // Abort any in-flight motion before closing — bypass CTS so the command
    // reaches the controller even while motors are running.
    if (m_hwFlowControl)
        m_port->setFlowControl(QSerialPort::NoFlowControl);
    m_port->write(mcl3(0x07, "a"));
    m_port->waitForBytesWritten(500);
    m_rxBuf.clear();

    m_port->close();
    m_connected = false;
    emit disconnected();
}

bool LStepStage::isConnected() const { return m_connected; }

// ---------------------------------------------------------------------------
// Initialization sequence (empirically verified on Walter Uhl MS4-WT02)
// ---------------------------------------------------------------------------

void LStepStage::sendInitSequence()
{
    // Step 1: XON announcement – not MCL3, must be sent as raw ASCII
    enqueueWrite(QByteArray("!cts 1\r"));

    // Steps 2–20: MCL3 register writes (no responses expected)
    enqueueWrite(mcl3(0x0B, "7"));         //  2: ActiveAxes = XYZ
    enqueueWrite(mcl3(0x09, "50"));        //  3: Speed = 50
    enqueueWrite(mcl3(0x08, "500"));       //  4: Ramp = 500
    enqueueWrite(mcl3(0x15, "20000"));     //  5: Pitch X = 2.0 mm
    enqueueWrite(mcl3(0x16, "20000"));     //  6: Pitch Y = 2.0 mm
    enqueueWrite(mcl3(0x17, "10000"));     //  7: Pitch Z = 1.0 mm
    enqueueWrite(mcl3(0x30, "100"));       //  8: MotorCurrent X = 100%
    enqueueWrite(mcl3(0x31, "100"));       //  9: MotorCurrent Y = 100%
    enqueueWrite(mcl3(0x32, "100"));       // 10: MotorCurrent Z = 100%
    enqueueWrite(mcl3(0x0A, "5"));         // 11: CurrentReduction = 5
    enqueueWrite(mcl3(0x2F, "65087"));     // 12: Init-mandatory (unknown purpose)
    enqueueWrite(mcl3(0x18, "000000000")); // 13: Init-mandatory (unknown purpose)
    enqueueWrite(mcl3(0x07, "d0x"));       // 14: SetTVRMode 0 for X
    enqueueWrite(mcl3(0x07, "d0y"));       // 15: SetTVRMode 0 for Y
    enqueueWrite(mcl3(0x07, "d0z"));       // 16: SetTVRMode 0 for Z
    enqueueWrite(mcl3(0x0B, "7"));         // 17: ActiveAxes = XYZ
    enqueueWrite(mcl3(0x23, "0"));         // 18: Trigger = off
    enqueueWrite(mcl3(0x08, "500"));       // 19: Ramp = 500
    enqueueWrite(mcl3(0x09, "50"));        // 20: Speed = 50

    // Initial position refresh
    enqueuePositionQuery();
}

// ---------------------------------------------------------------------------
// Movement commands
// ---------------------------------------------------------------------------

void LStepStage::moveAbsolute(double swX, double swY, double swZ)
{
    if (!m_connected) return;

    const long hwX = mmToUnits(swToHwX(swX));
    const long hwY = mmToUnits(swToHwY(swY));
    const long hwZ = mmToUnits(swToHwZ(swZ));

    // Use homing-proven Z parameters whenever Z is part of the move.
    const bool movesZ = (std::abs(swZ - m_position.z) > 0.0001);
    if (movesZ) {
        enqueueWrite(mcl3(0x09, "100"));  // Speed = 100 (same as calibrate)
        enqueueWrite(mcl3(0x08, "50"));   // Ramp  = 50  (gentle enough for EM brake)
    }

    enqueueWrite(mcl3(0x00, QString::number(hwX)));  // Preselection X
    enqueueWrite(mcl3(0x01, QString::number(hwY)));  // Preselection Y
    enqueueWrite(mcl3(0x02, QString::number(hwZ)));  // Preselection Z
    enqueueWrite(mcl3(0x0B, "7"));                   // ActiveAxes = XYZ

    enqueueMove(mcl3(0x07, "r"),                     // Sub-command 'r' = MoveAbsolute
        [this, movesZ](const QByteArray &resp) {
            if (movesZ) {
                enqueueWrite(mcl3(0x09, "50"));   // Restore Speed
                enqueueWrite(mcl3(0x08, "500"));  // Restore Ramp
            }
            emit movementFinished(QString::fromLatin1(resp));
            enqueuePositionQuery();
        }, 30000);
}

void LStepStage::moveRelative(double swDx, double swDy, double swDz)
{
    if (!m_connected) return;

    // All software axes are mirrored/inverted relative to hardware,
    // so a positive software delta maps to a negative hardware delta.
    const long hwDx = mmToUnits(-swDx);
    const long hwDy = mmToUnits(-swDy);
    const long hwDz = mmToUnits(-swDz);

    // Z-only moves use reduced speed/ramp so the electromagnetic brake can disengage.
    const bool zOnly = (swDx == 0.0 && swDy == 0.0 && swDz != 0.0);

    if (zOnly) {
        enqueueWrite(mcl3(0x09, "20"));                  // Speed = 20 U/s
        enqueueWrite(mcl3(0x08, "200"));                 // Ramp  = 200 (doc-recommended for Z brake)
        enqueueWrite(mcl3(0x02, QString::number(hwDz))); // Preselection Z
        enqueueWrite(mcl3(0x0B, "4"));                   // ActiveAxes = Z only
    } else {
        enqueueWrite(mcl3(0x00, QString::number(hwDx)));
        enqueueWrite(mcl3(0x01, QString::number(hwDy)));
        enqueueWrite(mcl3(0x02, QString::number(hwDz)));
        enqueueWrite(mcl3(0x0B, "7"));                   // ActiveAxes = XYZ
    }

    enqueueMove(mcl3(0x07, "v"),                         // 'v' = MoveRelative
        [this, zOnly](const QByteArray &resp) {
            if (zOnly) {
                enqueueWrite(mcl3(0x09, "50"));   // Restore Speed = 50
                enqueueWrite(mcl3(0x08, "500"));  // Restore Ramp  = 500
                enqueueWrite(mcl3(0x0B, "7"));    // Restore ActiveAxes = XYZ
            }
            emit movementFinished(QString::fromLatin1(resp));
            enqueuePositionQuery();
        }, 30000);
}

void LStepStage::calibrate()
{
    if (!m_connected) return;

    // Calibrate: faster speed, gentler ramp to avoid missing the switch
    enqueueWrite(mcl3(0x09, "100"));   // Speed = 100
    enqueueWrite(mcl3(0x08, "50"));    // Ramp  = 50
    enqueueWrite(mcl3(0x0B, "7"));     // ActiveAxes = all

    enqueueMove(mcl3(0x07, "c"),       // 'c' = Calibrate (home)
        [this](const QByteArray &resp) {
            // After homing, Z is parked on the null switch with the EM brake fully
            // engaged.  Any subsequent caller using default Speed/Ramp (500) will
            // stall Z because the ramp is too steep for the brake to release.
            // Move Z 1 mm away from the switch (+1000 hardware units = +1 mm in the
            // hardware-positive/stage-down direction) so the brake disengages and the
            // controller is in the same state as after a power-cycle (stage mid-range).
            enqueueWrite(mcl3(0x09, "20"));              // Speed = 20 (slow, brake-safe)
            enqueueWrite(mcl3(0x08, "200"));             // Ramp  = 200 (gentle)
            enqueueWrite(mcl3(0x0B, "4"));               // ActiveAxes = Z only
            enqueueWrite(mcl3(0x02, QString::number(mmToUnits(1.0))));  // Z presel = +1 mm
            enqueueMove(mcl3(0x07, "v"),                 // 'v' = MoveRelative
                [this, resp](const QByteArray &) {
                    enqueueWrite(mcl3(0x09, "50"));      // Restore Speed
                    enqueueWrite(mcl3(0x08, "500"));     // Restore Ramp
                    enqueueWrite(mcl3(0x0B, "7"));       // Restore ActiveAxes = XYZ
                    m_position = {0, 0, 0};
                    emit positionChanged(0, 0, 0);
                    emit movementFinished(QString::fromLatin1(resp));
                    // Poll timer will update position display within 500 ms
                }, 10000);
        }, 60000);
}

void LStepStage::measureLength()
{
    if (!m_connected) return;

    // Z has an EM brake that requires the same slow-ramp parameters as calibrate.
    // Without these, Speed=50/Ramp=500 is too aggressive for the brake to disengage
    // and Z stalls, leaving the 'l' command waiting forever.
    enqueueWrite(mcl3(0x09, "100"));  // Speed = 100 (same as calibrate)
    enqueueWrite(mcl3(0x08, "50"));   // Ramp  = 50  (gentle enough for EM brake)
    enqueueWrite(mcl3(0x0B, "7"));    // ActiveAxes = XYZ
    enqueueMove(mcl3(0x07, "l"),       // 'l' = MeasureLength (drive to end-switches)
        [this](const QByteArray &resp) {
            enqueueWrite(mcl3(0x09, "50"));   // Restore Speed
            enqueueWrite(mcl3(0x08, "500"));  // Restore Ramp
            // Read raw positions immediately after end-switches hit.
            // UC, UD, UE are read-address commands (register 0x43/0x44/0x45).
            enqueueRead(QByteArray("\x55\x43\x0D", 3),
                [this](const QByteArray &rx) {
                    m_hwMax.x = unitsToMm(rx.trimmed().toLong());
                    enqueueRead(QByteArray("\x55\x44\x0D", 3),
                        [this](const QByteArray &ry) {
                            m_hwMax.y = unitsToMm(ry.trimmed().toLong());
                            enqueueRead(QByteArray("\x55\x45\x0D", 3),
                                [this](const QByteArray &rz) {
                                    m_hwMax.z = unitsToMm(rz.trimmed().toLong());
                                    enqueuePositionQuery();
                                });
                        });
                });
            emit movementFinished(QString::fromLatin1(resp));
        }, 60000);
}

void LStepStage::abort()
{
    if (!m_connected) return;

    m_cmdTimeout->stop();
    m_queue.clear();
    m_busy = false;
    m_rxBuf.clear();

    // The controller deasserts CTS while motors are running, which would
    // block a normal write. Temporarily disable flow control so the abort
    // reaches the controller immediately.
    if (m_hwFlowControl)
        m_port->setFlowControl(QSerialPort::NoFlowControl);
    m_port->write(mcl3(0x07, "a"));
    m_port->waitForBytesWritten(500);
    if (m_hwFlowControl)
        m_port->setFlowControl(QSerialPort::HardwareControl);
}

// ---------------------------------------------------------------------------
// Position query (chains X → Y → Z)
// ---------------------------------------------------------------------------

void LStepStage::enqueuePositionQuery()
{
    // UC\r = 55 43 0D (read X absolute position)
    enqueueRead(QByteArray("\x55\x43\x0D", 3),
        [this](const QByteArray &rx) {
            const double hwX = unitsToMm(rx.trimmed().toLong());
            // UD\r = 55 44 0D
            enqueueRead(QByteArray("\x55\x44\x0D", 3),
                [this, hwX](const QByteArray &ry) {
                    const double hwY = unitsToMm(ry.trimmed().toLong());
                    // UE\r = 55 45 0D
                    enqueueRead(QByteArray("\x55\x45\x0D", 3),
                        [this, hwX, hwY](const QByteArray &rz) {
                            const double hwZ = unitsToMm(rz.trimmed().toLong());
                            m_position = {hwToSwX(hwX), hwToSwY(hwY), hwToSwZ(hwZ)};
                            emit positionChanged(m_position.x, m_position.y, m_position.z);
                        });
                });
        });
}

void LStepStage::queryPosition()
{
    if (m_connected) enqueuePositionQuery();
}

// ---------------------------------------------------------------------------
// Pitch override
// ---------------------------------------------------------------------------

void LStepStage::setPitch(int32_t xP, int32_t yP, int32_t zP)
{
    enqueueWrite(mcl3(0x15, QString::number(xP)));
    enqueueWrite(mcl3(0x16, QString::number(yP)));
    enqueueWrite(mcl3(0x17, QString::number(zP)));
}

// ---------------------------------------------------------------------------
// Queue machinery
// ---------------------------------------------------------------------------

void LStepStage::enqueueWrite(const QByteArray &bytes, int timeoutMs)
{
    Cmd cmd;
    cmd.bytes             = bytes;
    cmd.expectsResponse   = false;
    cmd.timeoutMs         = timeoutMs;
    m_queue.enqueue(cmd);
    if (!m_busy)
        QMetaObject::invokeMethod(this, &LStepStage::pumpQueue, Qt::QueuedConnection);
}

void LStepStage::enqueueRead(const QByteArray &bytes,
                              std::function<void(const QByteArray &)> handler,
                              int timeoutMs)
{
    Cmd cmd;
    cmd.bytes           = bytes;
    cmd.expectsResponse = true;
    cmd.handler         = std::move(handler);
    cmd.timeoutMs       = timeoutMs;
    m_queue.enqueue(cmd);
    if (!m_busy)
        QMetaObject::invokeMethod(this, &LStepStage::pumpQueue, Qt::QueuedConnection);
}

void LStepStage::enqueueMove(const QByteArray &subCmdBytes,
                              std::function<void(const QByteArray &)> handler,
                              int timeoutMs)
{
    // Write the sub-command register (no response), then send UP and wait
    enqueueWrite(subCmdBytes);
    enqueueRead(upCmd(), std::move(handler), timeoutMs);
}

void LStepStage::pumpQueue()
{
    if (m_busy || m_queue.isEmpty()) return;

    const Cmd &cmd = m_queue.head();

    m_port->write(cmd.bytes);
    m_port->waitForBytesWritten(2000);

    if (cmd.expectsResponse) {
        m_busy = true;
        m_cmdTimeout->start(cmd.timeoutMs);
    } else {
        m_queue.dequeue();
        // Schedule next command via event loop (avoids deep call stack)
        QMetaObject::invokeMethod(this, &LStepStage::pumpQueue, Qt::QueuedConnection);
    }
}

// ---------------------------------------------------------------------------
// Serial read and timeout
// ---------------------------------------------------------------------------

void LStepStage::onReadyRead()
{
    m_rxBuf.append(m_port->readAll());

    const int crIdx = m_rxBuf.indexOf('\r');
    if (crIdx < 0) return; // incomplete packet – wait for more bytes

    const QByteArray response = m_rxBuf.left(crIdx);
    m_rxBuf.remove(0, crIdx + 1);

    finishCurrent(response);
}

void LStepStage::onCommandTimeout()
{
    emit errorOccurred(QStringLiteral("LStep command timeout"));
    // Abort the controller immediately so the timed-out command's UP response
    // never arrives and corrupts the next queued read (protocol desync).
    m_port->write(mcl3(0x07, "a"));
    m_port->waitForBytesWritten(500);
    m_rxBuf.clear();
    finishCurrent(QByteArray("TIMEOUT"));
}

void LStepStage::finishCurrent(const QByteArray &response)
{
    if (!m_busy || m_queue.isEmpty()) return;

    m_cmdTimeout->stop();
    const Cmd cmd = m_queue.dequeue();
    m_busy = false;

    if (cmd.handler)
        cmd.handler(response);

    QMetaObject::invokeMethod(this, &LStepStage::pumpQueue, Qt::QueuedConnection);
}

// ---------------------------------------------------------------------------
// Polling
// ---------------------------------------------------------------------------

void LStepStage::onPollTimer()
{
    if (!m_busy && m_queue.isEmpty())
        enqueuePositionQuery();
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

StagePosition LStepStage::position()    const { return m_position; }
StagePosition LStepStage::travelRange() const { return m_hwMax;    }

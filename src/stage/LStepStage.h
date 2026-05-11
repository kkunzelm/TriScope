#pragma once

#include "interfaces/IPositioningStage.h"
#include <QSerialPort>
#include <QQueue>
#include <QTimer>
#include <QByteArray>
#include <functional>

// Lang LStep 23 / MCL3 serial protocol implementation.
//
// All movement methods are non-blocking. They enqueue MCL3 commands in a
// FIFO queue that is drained one command at a time, using Qt's event loop
// for async read completion.
//
// Coordinate transform:
//   Hardware origin = right-back-top (nullswitches).
//   Software origin = left-front-top (right-handed, +Z upward).
//   Transform: swX = hwMaxX - hwX, swY = hwMaxY - hwY, swZ = -hwZ.
class LStepStage final : public IPositioningStage
{
    Q_OBJECT
public:
    explicit LStepStage(QObject *parent = nullptr);
    ~LStepStage() override;

    bool connect(const QString &port) override;
    void disconnect()                 override;
    bool isConnected() const          override;

    void moveAbsolute(double x, double y, double z)    override;
    void moveRelative(double dx, double dy, double dz) override;
    void calibrate()  override;
    void setHome()    override;
    bool isHomed()    const override { return m_homed; }
    void abort()      override;

    StagePosition position()    const override;
    StagePosition travelRange() const override;

    // Override pitch registers after connection (default: MS4-WT02 values).
    void setPitch(int32_t xPitch10kMm, int32_t yPitch10kMm, int32_t zPitch10kMm);

    // Drive axes to end-switches to measure travel range. Call after calibrate().
    void measureLength();

    // Request an immediate position refresh.
    void queryPosition();

private slots:
    void onReadyRead();
    void onCommandTimeout();
    void pumpQueue();
    void onPollTimer();

private:
    // ---- Command queue ----
    struct Cmd {
        QByteArray bytes;
        bool       expectsResponse = false;
        std::function<void(const QByteArray &)> handler;
        int timeoutMs = 5000;
    };

    void enqueueWrite(const QByteArray &bytes, int timeoutMs = 5000);
    void enqueueRead (const QByteArray &bytes,
                      std::function<void(const QByteArray &)> handler,
                      int timeoutMs = 5000);
    // Shortcut: write sub-command reg7 + enqueue "UP" read
    void enqueueMove (const QByteArray &subCmdBytes,
                      std::function<void(const QByteArray &)> handler = nullptr,
                      int timeoutMs = 30000);

    void sendNext();
    void finishCurrent(const QByteArray &response);

    // ---- Initialization ----
    void sendInitSequence();

    // ---- Position queries (chain X→Y→Z) ----
    void enqueuePositionQuery();

    // ---- Coordinate helpers ----
    double hwToSwX(double hw) const { return m_hwRef.x - hw; }
    double hwToSwY(double hw) const { return m_hwRef.y - hw; }
    double hwToSwZ(double hw) const { return m_hwRef.z - hw; }
    double swToHwX(double sw) const { return m_hwRef.x - sw; }
    double swToHwY(double sw) const { return m_hwRef.y - sw; }
    double swToHwZ(double sw) const { return m_hwRef.z - sw; }

    static long   mmToUnits(double mm)  { return static_cast<long>(std::llround(mm * 1000.0)); }
    static double unitsToMm(long units) { return units / 1000.0; }

    // Build MCL3 command: 0x55 + reg (binary) + value (ASCII) + 0x0D
    static QByteArray mcl3(uint8_t reg, const QString &value = {});
    // UP command bytes (0x55 0x50 0x0D)
    static QByteArray upCmd();

    // ---- Members ----
    QSerialPort *m_port        = nullptr;
    QTimer      *m_cmdTimeout  = nullptr;
    QTimer      *m_pollTimer   = nullptr;

    QQueue<Cmd> m_queue;
    bool        m_busy = false;
    QByteArray  m_rxBuf;

    StagePosition m_position{};
    // Hardware position that maps to software (0,0,0). Set by calibrate() or setHome().
    // Initial values match the pre-calibration default so behaviour is unchanged
    // before the first calibrate() call: X/Y mirror via hwRef, Z uses hwRef.z=0 → swZ=-hwZ.
    StagePosition m_hwRef{165.8, 166.5, 0.0};
    StagePosition m_hwRange{165.8, 166.5, 104.6}; // physical travel range (mm), updated by measureLength()

    bool m_connected      = false;
    bool m_homed          = false;
    bool m_hwFlowControl  = false;
};

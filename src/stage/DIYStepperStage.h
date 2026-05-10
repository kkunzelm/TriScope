#pragma once

#include "interfaces/IPositioningStage.h"

// Stub implementation for a future DIY G-code stepper controller.
// All methods emit errorOccurred("not implemented") except isConnected().
class DIYStepperStage final : public IPositioningStage
{
    Q_OBJECT
public:
    explicit DIYStepperStage(QObject *parent = nullptr);

    bool connect(const QString &port) override;
    void disconnect()                 override;
    bool isConnected() const          override;

    void moveAbsolute(double x, double y, double z)    override;
    void moveRelative(double dx, double dy, double dz) override;
    void calibrate()  override;
    void abort()      override;

    StagePosition position()    const override;
    StagePosition travelRange() const override;

private:
    bool m_connected = false;
};

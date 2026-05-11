#pragma once

#include <QObject>
#include <QString>

// All positions in mm. Right-handed coordinate system:
//   Origin (0,0,0) = Front-Left corner from user perspective.
//   +X: right, +Y: away from user (into table), +Z: upward (towards objective).
struct StagePosition {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// Hardware-independent positioning stage interface.
// Concrete implementations: LStepStage, DIYStepperStage.
// All movement methods are non-blocking; completion is signaled via movementFinished().
class IPositioningStage : public QObject
{
    Q_OBJECT
public:
    explicit IPositioningStage(QObject *parent = nullptr) : QObject(parent) {}
    ~IPositioningStage() override = default;

    virtual bool connect(const QString &port) = 0;
    virtual void disconnect()                 = 0;
    virtual bool isConnected() const          = 0;

    // Move to absolute software coordinates (mm).
    virtual void moveAbsolute(double x, double y, double z) = 0;

    // Move by delta relative to current position (mm).
    virtual void moveRelative(double dx, double dy, double dz) = 0;

    // Drive all axes to their home (zero) switches. Long operation – 60 s timeout.
    virtual void calibrate() = 0;

    // Declare the current physical position as software (0,0,0). Pure software
    // operation — no movement. Emits positionChanged(0,0,0) when done.
    virtual void setHome() = 0;

    // Emergency stop. Interrupts any active motion immediately.
    virtual void abort() = 0;

    virtual StagePosition position()    const = 0;  // last known software position
    virtual StagePosition travelRange() const = 0;  // max X/Y/Z in mm

signals:
    void positionChanged(double x, double y, double z);
    void movementFinished(const QString &status);
    void errorOccurred(const QString &message);
    void connected();
    void disconnected();
};

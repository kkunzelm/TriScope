#include "stage/DIYStepperStage.h"

DIYStepperStage::DIYStepperStage(QObject *parent)
    : IPositioningStage(parent)
{}

bool DIYStepperStage::connect(const QString &)
{
    emit errorOccurred(QStringLiteral("DIY stepper: not yet implemented"));
    return false;
}

void DIYStepperStage::disconnect() { m_connected = false; emit disconnected(); }
bool DIYStepperStage::isConnected() const { return m_connected; }

void DIYStepperStage::moveAbsolute(double, double, double)
    { emit errorOccurred(QStringLiteral("DIY stepper: not yet implemented")); }
void DIYStepperStage::moveRelative(double, double, double)
    { emit errorOccurred(QStringLiteral("DIY stepper: not yet implemented")); }
void DIYStepperStage::calibrate()
    { emit errorOccurred(QStringLiteral("DIY stepper: not yet implemented")); }
void DIYStepperStage::setHome()
    { emit errorOccurred(QStringLiteral("DIY stepper: not yet implemented")); }
void DIYStepperStage::abort()
    { emit errorOccurred(QStringLiteral("DIY stepper: not yet implemented")); }

StagePosition DIYStepperStage::position()    const { return {}; }
StagePosition DIYStepperStage::travelRange() const { return {}; }

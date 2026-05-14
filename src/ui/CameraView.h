#pragma once

#include <QWidget>
#include <QImage>
#include <QVector>
#include <QPointF>
#include "ui/MeasurementOverlay.h"

// Displays live camera frames with optional overlays:
//   - Crosshair (permanent centre lines)
//   - 10×10 reference grid
//   - Interactive measurement tools (Distance / Angle / Radius)
//
// Frame updates arrive from AcquisitionThread via a queued signal and
// trigger an asynchronous repaint – the acquisition thread never blocks
// waiting for the UI.
class CameraView : public QWidget
{
    Q_OBJECT
public:
    explicit CameraView(QWidget *parent = nullptr);

    void setCrosshairVisible(bool v);
    void setGridVisible(bool v);
    void startMeasurement(MeasurementOverlay::Mode mode);
    void clearMeasurements();
    void setScale(double umPerPixel);

public slots:
    void onFrameReady(const QImage &frame);
    // Points in original image-pixel coordinates (before the scanner's 90° CCW rotation).
    // gauss = green, cog = yellow; cleared automatically when scanActiveChanged fires.
    void setLaserOverlay(const QVector<QPointF> &gauss, const QVector<QPointF> &cog);
    void clearLaserOverlay();

signals:
    void measurementResult(const QString &text);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    void drawCrosshair(QPainter &p) const;
    void drawGrid(QPainter &p) const;
    void drawLaserOverlay(QPainter &p) const;

    QPointF widgetToImage(const QPointF &widgetPt) const;
    QPointF imageToWidget(const QPointF &imgPt) const;
    QRectF  imageRect() const;

    QImage             m_frame;
    bool               m_showCrosshair = false;
    bool               m_showGrid      = false;
    MeasurementOverlay m_overlay;
    QVector<QPointF>   m_gaussPoints;  // image-pixel coords, drawn green
    QVector<QPointF>   m_cogPoints;    // image-pixel coords, drawn yellow
};

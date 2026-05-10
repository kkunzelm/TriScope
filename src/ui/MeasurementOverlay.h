#pragma once

#include <QObject>
#include <QPointF>
#include <QVector>
#include <QPainter>
#include <optional>

// Stores state for one interactive measurement (Distance / Angle / Radius).
// The CameraView owns and paints this overlay; it notifies CameraView of
// repaints via the changed() signal.
class MeasurementOverlay : public QObject
{
    Q_OBJECT
public:
    enum class Mode { None, Distance, Angle, Radius };
    enum class State { Idle, WaitPoint1, WaitPoint2, WaitPoint3, Done };

    explicit MeasurementOverlay(QObject *parent = nullptr);

    void startTool(Mode mode);
    void clear();

    // Feed mouse clicks (in image-pixel coordinates) to the active tool.
    void addPoint(const QPointF &imagePt);

    // Calibration: set the scale factor (µm per image pixel).
    void setScale(double umPerPixel);
    double scale() const { return m_umPerPixel; }

    // Paint the overlay onto the widget. Call from CameraView::paintEvent.
    // widgetRect is the rectangle occupied by the scaled image inside the widget.
    void paint(QPainter &painter, const QRectF &widgetRect,
               const QSizeF &imageSize) const;

    State state() const { return m_state; }
    Mode  mode()  const { return m_mode;  }

signals:
    void changed();       // request repaint
    void resultReady(const QString &description);

private:
    QPointF imageToWidget(const QPointF &pt, const QRectF &wr,
                          const QSizeF &imgSz) const;
    void compute();
    void paintDistance(QPainter &, const QRectF &, const QSizeF &) const;
    void paintAngle   (QPainter &, const QRectF &, const QSizeF &) const;
    void paintRadius  (QPainter &, const QRectF &, const QSizeF &) const;

    Mode   m_mode  = Mode::None;
    State  m_state = State::Idle;
    QVector<QPointF> m_points;   // image-pixel coordinates
    double m_umPerPixel = 1.0;   // calibration scale
    QString m_resultText;
};

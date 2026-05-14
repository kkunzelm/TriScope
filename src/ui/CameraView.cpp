#include "ui/CameraView.h"

#include <QPainter>
#include <QPen>
#include <QMouseEvent>
#include <QResizeEvent>

CameraView::CameraView(QWidget *parent)
    : QWidget(parent)
    , m_overlay(this)
{
    setMinimumSize(320, 240);
    setAttribute(Qt::WA_OpaquePaintEvent);
    setMouseTracking(true);

    QObject::connect(&m_overlay, &MeasurementOverlay::changed,
                     this, QOverload<>::of(&CameraView::update));
    QObject::connect(&m_overlay, &MeasurementOverlay::resultReady,
                     this, &CameraView::measurementResult);
}

// ---------------------------------------------------------------------------
// Public control
// ---------------------------------------------------------------------------

void CameraView::setCrosshairVisible(bool v) { m_showCrosshair = v; update(); }
void CameraView::setGridVisible(bool v)      { m_showGrid = v;      update(); }

void CameraView::setLaserOverlay(const QVector<QPointF> &gauss, const QVector<QPointF> &cog)
{
    m_gaussPoints = gauss;
    m_cogPoints   = cog;
    update();
}

void CameraView::clearLaserOverlay()
{
    m_gaussPoints.clear();
    m_cogPoints.clear();
    update();
}

void CameraView::startMeasurement(MeasurementOverlay::Mode mode)
{
    m_overlay.startTool(mode);
    setCursor(mode == MeasurementOverlay::Mode::None ? Qt::ArrowCursor
                                                     : Qt::CrossCursor);
}

void CameraView::clearMeasurements()
{
    m_overlay.clear();
    setCursor(Qt::ArrowCursor);
}

void CameraView::setScale(double umPerPixel)
{
    m_overlay.setScale(umPerPixel);
}

// ---------------------------------------------------------------------------
// Frame slot (called via queued connection from AcquisitionThread)
// ---------------------------------------------------------------------------

void CameraView::onFrameReady(const QImage &frame)
{
    m_frame = frame;
    update(); // schedules repaint on next event loop tick
}

// ---------------------------------------------------------------------------
// Layout helpers
// ---------------------------------------------------------------------------

QRectF CameraView::imageRect() const
{
    if (m_frame.isNull())
        return rect().toRectF();

    const QSizeF imgSz = m_frame.size();
    const QSizeF widSz = size();

    const double scale = std::min(widSz.width()  / imgSz.width(),
                                  widSz.height() / imgSz.height());
    const QSizeF scaled(imgSz.width() * scale, imgSz.height() * scale);

    return QRectF(QPointF((widSz.width()  - scaled.width())  / 2.0,
                          (widSz.height() - scaled.height()) / 2.0),
                  scaled);
}

QPointF CameraView::widgetToImage(const QPointF &wp) const
{
    const QRectF ir = imageRect();
    if (m_frame.isNull() || ir.width() < 1 || ir.height() < 1)
        return wp;
    return QPointF((wp.x() - ir.left()) * m_frame.width()  / ir.width(),
                   (wp.y() - ir.top())  * m_frame.height() / ir.height());
}

QPointF CameraView::imageToWidget(const QPointF &imgPt) const
{
    const QRectF ir = imageRect();
    if (m_frame.isNull() || ir.width() < 1 || ir.height() < 1)
        return imgPt;
    return QPointF(ir.left() + imgPt.x() * ir.width()  / m_frame.width(),
                   ir.top()  + imgPt.y() * ir.height() / m_frame.height());
}

// ---------------------------------------------------------------------------
// Painting
// ---------------------------------------------------------------------------

void CameraView::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::SmoothPixmapTransform);

    // Background
    p.fillRect(rect(), Qt::black);

    // Frame
    if (!m_frame.isNull()) {
        const QRectF ir = imageRect();
        p.drawImage(ir, m_frame, m_frame.rect());
    }

    const QRectF ir = imageRect();
    const QSizeF imgSz = m_frame.isNull() ? QSizeF(ir.size()) : QSizeF(m_frame.size());

    if (m_showGrid)      drawGrid(p);
    if (m_showCrosshair) drawCrosshair(p);
    drawLaserOverlay(p);

    m_overlay.paint(p, ir, imgSz);
}

void CameraView::drawCrosshair(QPainter &p) const
{
    const QPointF centre = rect().center();
    const double cx = centre.x(), cy = centre.y();

    // Dark shadow pass (readability on bright backgrounds)
    p.setPen(QPen(QColor(0, 0, 0, 160), 3.0, Qt::SolidLine));
    p.drawLine(QPointF(0, cy),     QPointF(width(), cy));
    p.drawLine(QPointF(cx, 0),     QPointF(cx, height()));

    // Bright yellow main line
    p.setPen(QPen(QColor(255, 230, 0, 230), 1.5, Qt::SolidLine));
    p.drawLine(QPointF(0, cy),     QPointF(width(), cy));
    p.drawLine(QPointF(cx, 0),     QPointF(cx, height()));

    // Centre mark: small unfilled circle with cross gap
    p.setPen(QPen(QColor(255, 230, 0, 230), 1.5));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(centre, 5.0, 5.0);
}

void CameraView::drawGrid(QPainter &p) const
{
    const int cols = 10;
    const int rows = 10;
    // Semi-transparent white – solid lines are more readable than dotted
    const QPen pen(QColor(255, 255, 255, 130), 1.0, Qt::SolidLine);
    p.setPen(pen);

    const double cw = static_cast<double>(width())  / cols;
    const double ch = static_cast<double>(height()) / rows;

    for (int c = 1; c < cols; ++c)
        p.drawLine(QPointF(c * cw, 0), QPointF(c * cw, height()));
    for (int r = 1; r < rows; ++r)
        p.drawLine(QPointF(0, r * ch), QPointF(width(), r * ch));
}

void CameraView::drawLaserOverlay(QPainter &p) const
{
    if (m_frame.isNull()) return;
    if (m_gaussPoints.isEmpty() && m_cogPoints.isEmpty()) return;

    // CoG first (yellow), then Gaussian on top (green) so green wins where they overlap.
    struct Layer { const QVector<QPointF> *pts; QColor color; };
    const Layer layers[] = {
        { &m_cogPoints,   QColor(255, 220,   0, 210) },
        { &m_gaussPoints, QColor(  0, 220,  60, 210) },
    };

    for (const auto &layer : layers) {
        if (layer.pts->isEmpty()) continue;
        QVector<QPointF> widgetPts;
        widgetPts.reserve(layer.pts->size());
        for (const QPointF &ip : *layer.pts)
            widgetPts.append(imageToWidget(ip));
        p.setPen(QPen(layer.color, 2.0, Qt::SolidLine, Qt::RoundCap));
        p.drawPoints(widgetPts.constData(), widgetPts.size());
    }
}

// ---------------------------------------------------------------------------
// Mouse input
// ---------------------------------------------------------------------------

void CameraView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() != Qt::LeftButton) return;
    if (m_overlay.state() == MeasurementOverlay::State::Idle) return;

    m_overlay.addPoint(widgetToImage(event->position()));
}

void CameraView::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    update();
}

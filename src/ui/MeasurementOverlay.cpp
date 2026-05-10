#include "ui/MeasurementOverlay.h"

#include <QPen>
#include <QFont>
#include <cmath>

MeasurementOverlay::MeasurementOverlay(QObject *parent)
    : QObject(parent)
{}

void MeasurementOverlay::startTool(Mode mode)
{
    m_mode      = mode;
    m_state     = (mode == Mode::None) ? State::Idle : State::WaitPoint1;
    m_points.clear();
    m_resultText.clear();
    emit changed();
}

void MeasurementOverlay::clear()
{
    m_mode  = Mode::None;
    m_state = State::Idle;
    m_points.clear();
    m_resultText.clear();
    emit changed();
}

void MeasurementOverlay::setScale(double umPerPixel) { m_umPerPixel = umPerPixel; }

// ---------------------------------------------------------------------------
// Point input state machine
// ---------------------------------------------------------------------------

void MeasurementOverlay::addPoint(const QPointF &imagePt)
{
    if (m_state == State::Idle) return;

    if (m_state == State::Done) {
        m_points.clear();
        m_resultText.clear();
        m_state = State::WaitPoint1;
    }

    m_points.append(imagePt);

    switch (m_mode) {
    case Mode::Distance:
        if (m_points.size() == 2) { m_state = State::Done; compute(); }
        else                        m_state = State::WaitPoint2;
        break;
    case Mode::Angle:
    case Mode::Radius:
        if (m_points.size() == 1)      m_state = State::WaitPoint2;
        else if (m_points.size() == 2) m_state = State::WaitPoint3;
        else                          { m_state = State::Done; compute(); }
        break;
    default: break;
    }

    emit changed();
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

static double pixelDist(const QPointF &a, const QPointF &b)
{
    return std::hypot(b.x() - a.x(), b.y() - a.y());
}

static double angleDeg(const QPointF &vertex, const QPointF &p1, const QPointF &p2)
{
    const QPointF v1 = p1 - vertex;
    const QPointF v2 = p2 - vertex;
    const double dot = v1.x() * v2.x() + v1.y() * v2.y();
    const double mag = std::hypot(v1.x(), v1.y()) * std::hypot(v2.x(), v2.y());
    if (mag < 1e-9) return 0.0;
    return std::acos(std::clamp(dot / mag, -1.0, 1.0)) * 180.0 / M_PI;
}

static double circumradius(const QPointF &a, const QPointF &b, const QPointF &c)
{
    const double ab = pixelDist(a, b);
    const double bc = pixelDist(b, c);
    const double ca = pixelDist(c, a);
    const double area = std::abs((b.x()-a.x())*(c.y()-a.y()) -
                                 (c.x()-a.x())*(b.y()-a.y())) / 2.0;
    return (area < 1e-9) ? 0.0 : (ab * bc * ca) / (4.0 * area);
}

void MeasurementOverlay::compute()
{
    if (m_state != State::Done) return;

    switch (m_mode) {
    case Mode::Distance: {
        const double px = pixelDist(m_points[0], m_points[1]);
        const double um = px * m_umPerPixel;
        m_resultText = QStringLiteral("Distance: %1 µm  (%2 px)")
            .arg(um, 0, 'f', 1).arg(px, 0, 'f', 1);
        break;
    }
    case Mode::Angle: {
        const double deg = angleDeg(m_points[0], m_points[1], m_points[2]);
        m_resultText = QStringLiteral("Angle: %1°").arg(deg, 0, 'f', 2);
        break;
    }
    case Mode::Radius: {
        const double r_px = circumradius(m_points[0], m_points[1], m_points[2]);
        const double r_um = r_px * m_umPerPixel;
        m_resultText = QStringLiteral("Radius: %1 µm  (%2 px)")
            .arg(r_um, 0, 'f', 1).arg(r_px, 0, 'f', 1);
        break;
    }
    default: break;
    }

    if (!m_resultText.isEmpty())
        emit resultReady(m_resultText);
}

// ---------------------------------------------------------------------------
// Coordinate transform: image pixels → widget pixels
// ---------------------------------------------------------------------------

QPointF MeasurementOverlay::imageToWidget(const QPointF &pt, const QRectF &wr,
                                          const QSizeF &imgSz) const
{
    if (imgSz.width() < 1 || imgSz.height() < 1) return pt;
    return QPointF(wr.left() + pt.x() * wr.width()  / imgSz.width(),
                   wr.top()  + pt.y() * wr.height() / imgSz.height());
}

// ---------------------------------------------------------------------------
// Painting helpers
// ---------------------------------------------------------------------------

static void drawShadowText(QPainter &p, const QPointF &pos, const QString &text)
{
    p.setPen(QColor(0, 0, 0, 210));
    p.drawText(pos + QPointF(1, 1), text);
    p.setPen(Qt::white);
    p.drawText(pos, text);
}

// ---------------------------------------------------------------------------
// Painting
// ---------------------------------------------------------------------------

void MeasurementOverlay::paint(QPainter &painter, const QRectF &widgetRect,
                                const QSizeF &imageSize) const
{
    if (m_mode == Mode::None || m_points.isEmpty()) return;

    switch (m_mode) {
    case Mode::Distance: paintDistance(painter, widgetRect, imageSize); break;
    case Mode::Angle:    paintAngle   (painter, widgetRect, imageSize); break;
    case Mode::Radius:   paintRadius  (painter, widgetRect, imageSize); break;
    default: break;
    }
}

void MeasurementOverlay::paintDistance(QPainter &p, const QRectF &wr,
                                        const QSizeF &sz) const
{
    const bool done = (m_state == State::Done);
    const QPen linePen(Qt::yellow, done ? 2.5 : 1.5, Qt::SolidLine);
    const QPen dotPen (Qt::red,    done ? 7   : 5,   Qt::SolidLine);
    p.setPen(dotPen);

    QVector<QPointF> wPts;
    for (const auto &ip : m_points)
        wPts.append(imageToWidget(ip, wr, sz));

    for (const auto &wp : wPts)
        p.drawPoint(wp);

    if (wPts.size() >= 2) {
        p.setPen(linePen);
        p.drawLine(wPts[0], wPts[1]);

        if (!m_resultText.isEmpty()) {
            const QPointF mid = (wPts[0] + wPts[1]) / 2.0;
            drawShadowText(p, mid + QPointF(4, -4), m_resultText);
        }
    }
}

void MeasurementOverlay::paintAngle(QPainter &p, const QRectF &wr,
                                     const QSizeF &sz) const
{
    const bool done = (m_state == State::Done);
    const QPen linePen(Qt::cyan, done ? 2.5 : 1.5);
    const QPen dotPen (Qt::red,  done ? 7   : 5);

    QVector<QPointF> wPts;
    for (const auto &ip : m_points)
        wPts.append(imageToWidget(ip, wr, sz));

    p.setPen(dotPen);
    for (const auto &wp : wPts) p.drawPoint(wp);

    if (wPts.size() >= 2) {
        p.setPen(linePen);
        p.drawLine(wPts[0], wPts[1]);
    }
    if (wPts.size() >= 3) {
        p.drawLine(wPts[0], wPts[2]);
        if (!m_resultText.isEmpty())
            drawShadowText(p, wPts[0] + QPointF(6, -6), m_resultText);
    }
}

void MeasurementOverlay::paintRadius(QPainter &p, const QRectF &wr,
                                      const QSizeF &sz) const
{
    const bool done = (m_state == State::Done);
    const QPen linePen(Qt::green, done ? 2.5 : 1.5);
    const QPen dotPen (Qt::red,   done ? 7   : 5);

    QVector<QPointF> wPts;
    for (const auto &ip : m_points)
        wPts.append(imageToWidget(ip, wr, sz));

    p.setPen(dotPen);
    for (const auto &wp : wPts) p.drawPoint(wp);

    p.setPen(linePen);
    for (int i = 1; i < wPts.size(); ++i)
        p.drawLine(wPts[i - 1], wPts[i]);

    if (wPts.size() == 3) {
        const double r_px = circumradius(m_points[0], m_points[1], m_points[2]);
        if (r_px > 0.5) {
            // Find circumcenter
            const QPointF &A = m_points[0], &B = m_points[1], &C = m_points[2];
            const double D = 2*(A.x()*(B.y()-C.y()) + B.x()*(C.y()-A.y()) + C.x()*(A.y()-B.y()));
            if (std::abs(D) > 1e-9) {
                const double ux = ((A.x()*A.x()+A.y()*A.y())*(B.y()-C.y()) +
                                   (B.x()*B.x()+B.y()*B.y())*(C.y()-A.y()) +
                                   (C.x()*C.x()+C.y()*C.y())*(A.y()-B.y())) / D;
                const double uy = ((A.x()*A.x()+A.y()*A.y())*(C.x()-B.x()) +
                                   (B.x()*B.x()+B.y()*B.y())*(A.x()-C.x()) +
                                   (C.x()*C.x()+C.y()*C.y())*(B.x()-A.x())) / D;

                const QPointF center = imageToWidget({ux, uy}, wr, sz);
                const double rw = r_px * wr.width() / sz.width();
                p.drawEllipse(center, rw, rw);

                if (!m_resultText.isEmpty())
                    drawShadowText(p, center + QPointF(4, -4), m_resultText);
            }
        }
    }
}

#include "robot_motion_test_app/graph_widget.h"

#include <QPainter>
#include <QPainterPath>
#include <QDebug>
#include <algorithm>
#include <cmath>
#include <limits>

// Creates the graph widget.
GraphWidget::GraphWidget(
    const QString& title,
    const QString& xLabel,
    const QString& yLabel,
    QWidget* parent)
    : QWidget(parent),
      title_(title),
      x_label_(xLabel),
      y_label_(yLabel)
{
    setMinimumSize(
        300,
        180);

    setAutoFillBackground(true);

    QPalette palette =
        this->palette();

    palette.setColor(
        QPalette::Window,
        Qt::white);

    setPalette(palette);
}

// Updates the plotted data.
void GraphWidget::setData(
    const QVector<QPointF>& data)
{
    data_ = data;

    qDebug() << "GraphWidget:"
             << title_
             << "received"
             << data_.size()
             << "points";

    update();
}

// Sets a vertical event marker on the graph.
void GraphWidget::setEventMarker(
    double x,
    const QString& label)
{
    if (!std::isfinite(x))
        return;

    EventMarker marker;
    marker.x = x;
    marker.label = label;

    event_markers_.append(marker);

    update();
}

// Removes the event marker.
void GraphWidget::clearEventMarker()
{
    event_markers_.clear();

    update();
}

// Draws the graph.
void GraphWidget::paintEvent(
    QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter painter(this);

    painter.setRenderHint(
        QPainter::Antialiasing,
        true);

    painter.fillRect(
        rect(),
        Qt::white);

    const int left =
        72;

    const int right =
        18;

    const int top =
        28;

    const int bottom =
        38;

    if (width() <= left + right ||
        height() <= top + bottom) {
        return;
    }

    const QRectF plotRect(
        left,
        top,
        width() - left - right,
        height() - top - bottom);

    double xMin = 0.0;
    double xMax = 1.0;
    double yMin = 0.0;
    double yMax = 1.0;

    if (!data_.isEmpty()) {

        xMin =
            std::numeric_limits<double>::max();

        xMax =
            std::numeric_limits<double>::lowest();

        yMin =
            std::numeric_limits<double>::max();

        yMax =
            std::numeric_limits<double>::lowest();

        for (const QPointF& point : data_) {

            if (!std::isfinite(point.x()) ||
                !std::isfinite(point.y())) {
                continue;
            }

            xMin =
                std::min(
                    xMin,
                    point.x());

            xMax =
                std::max(
                    xMax,
                    point.x());

            yMin =
                std::min(
                    yMin,
                    point.y());

            yMax =
                std::max(
                    yMax,
                    point.y());
        }

        if (xMin ==
                std::numeric_limits<double>::max() ||
            xMax ==
                std::numeric_limits<double>::lowest()) {

            xMin = 0.0;
            xMax = 1.0;
            yMin = 0.0;
            yMax = 1.0;
        }
    }

    if (std::abs(xMax - xMin) < 1e-9) {

        if (std::abs(xMax) < 1e-9) {
            xMax = 1.0;
        }
        else {
            const double margin =
                std::abs(xMax) * 0.1;

            xMin -= margin;
            xMax += margin;
        }
    }

    if (std::abs(yMax - yMin) < 1e-9) {

        if (std::abs(yMax) < 1e-9) {
            yMax = 1.0;
        }
        else {
            const double margin =
                std::abs(yMax) * 0.1;

            yMin -= margin;
            yMax += margin;
        }
    }

    const double xPadding =
        (xMax - xMin) * 0.05;

    const double yPadding =
        (yMax - yMin) * 0.10;

    xMin -= xPadding;
    xMax += xPadding;

    yMin -= yPadding;
    yMax += yPadding;

    if (xMin > 0.0)
        xMin = 0.0;

    if (yMin > 0.0)
        yMin = 0.0;

    drawAxes(
        painter,
        plotRect,
        xMin,
        xMax,
        yMin,
        yMax);

    drawData(
        painter,
        plotRect,
        xMin,
        xMax,
        yMin,
        yMax);

    drawEventMarker(
        painter,
        plotRect,
        xMin,
        xMax);

    painter.setPen(
        Qt::black);

    painter.setFont(
        QFont(
            "DejaVu Sans",
            10,
            QFont::DemiBold));

    painter.drawText(
        QRectF(
            0,
            4,
            width(),
            20),
        Qt::AlignCenter,
        title_);

    if (!x_label_.isEmpty()) {

        painter.setFont(
            QFont(
                "DejaVu Sans",
                8));

        painter.drawText(
            QRectF(
                left,
                height() - 20,
                plotRect.width(),
                16),
            Qt::AlignCenter,
            x_label_);
    }

    if (!y_label_.isEmpty()) {

        painter.save();

        painter.translate(
            11,
            plotRect.center().y());

        painter.rotate(-90.0);

        painter.setFont(
            QFont(
                "DejaVu Sans",
                8));

        painter.drawText(
            QRectF(
                -plotRect.height() / 2.0,
                0,
                plotRect.height(),
                16),
            Qt::AlignCenter,
            y_label_);

        painter.restore();
    }
}

// Draws PlotJuggler-style axes and ticks.
void GraphWidget::drawAxes(
    QPainter& painter,
    const QRectF& plotRect,
    double xMin,
    double xMax,
    double yMin,
    double yMax)
{
    const QColor axisColor(
        55,
        55,
        55);

    const QColor tickColor(
        90,
        90,
        90);

    const QColor gridColor(
        225,
        225,
        225);

    painter.setRenderHint(
        QPainter::Antialiasing,
        false);

    painter.setPen(
        QPen(
            axisColor,
            1.4));

    painter.drawLine(
        plotRect.left(),
        plotRect.top(),
        plotRect.left(),
        plotRect.bottom());

    painter.drawLine(
        plotRect.left(),
        plotRect.bottom(),
        plotRect.right(),
        plotRect.bottom());

    const double xRange =
        xMax - xMin;

    const double yRange =
        yMax - yMin;

    const double xStep =
        niceStep(xRange);

    const double yStep =
        niceStep(yRange);

    painter.setFont(
        QFont(
            "DejaVu Sans",
            7));

    // Draw vertical grid lines.
    painter.setPen(
        QPen(
            gridColor,
            1));

    const double firstX =
        std::ceil(xMin / xStep) *
        xStep;

    for (double value = firstX;
         value <= xMax + xStep * 0.01;
         value += xStep) {

        const double ratio =
            (value - xMin) / xRange;

        const double x =
            plotRect.left() +
            ratio *
                plotRect.width();

        painter.drawLine(
            QPointF(
                x,
                plotRect.top()),
            QPointF(
                x,
                plotRect.bottom()));
    }

    // Draw horizontal grid lines.
    const double firstY =
        std::ceil(yMin / yStep) *
        yStep;

    for (double value = firstY;
         value <= yMax + yStep * 0.01;
         value += yStep) {

        const double ratio =
            (value - yMin) / yRange;

        const double y =
            plotRect.bottom() -
            ratio *
                plotRect.height();

        painter.drawLine(
            QPointF(
                plotRect.left(),
                y),
            QPointF(
                plotRect.right(),
                y));
    }

    // Redraw the main axes.
    painter.setPen(
        QPen(
            axisColor,
            1.4));

    painter.drawLine(
        plotRect.left(),
        plotRect.top(),
        plotRect.left(),
        plotRect.bottom());

    painter.drawLine(
        plotRect.left(),
        plotRect.bottom(),
        plotRect.right(),
        plotRect.bottom());

    // Draw X-axis tick labels.
    const double firstXTick =
        std::ceil(xMin / xStep) *
        xStep;

    for (double value = firstXTick;
         value <= xMax + xStep * 0.01;
         value += xStep) {

        const double ratio =
            (value - xMin) / xRange;

        const double x =
            plotRect.left() +
            ratio *
                plotRect.width();

        painter.setPen(
            QPen(
                tickColor,
                1));

        painter.drawLine(
            QPointF(
                x,
                plotRect.bottom()),
            QPointF(
                x,
                plotRect.bottom() + 5));

        painter.setPen(
            Qt::black);

        painter.drawText(
            QRectF(
                x - 30,
                plotRect.bottom() + 6,
                60,
                14),
            Qt::AlignCenter,
            formatValue(value));
    }

    // Draw Y-axis tick labels.
    const double firstYTick =
        std::ceil(yMin / yStep) *
        yStep;

    for (double value = firstYTick;
         value <= yMax + yStep * 0.01;
         value += yStep) {

        const double ratio =
            (value - yMin) / yRange;

        const double y =
            plotRect.bottom() -
            ratio *
                plotRect.height();

        painter.setPen(
            QPen(
                tickColor,
                1));

        painter.drawLine(
            QPointF(
                plotRect.left() - 5,
                y),
            QPointF(
                plotRect.left(),
                y));

        painter.setPen(
            Qt::black);

        painter.drawText(
            QRectF(
                30,
                y - 8,
                plotRect.left() - 38,
                16),
            Qt::AlignRight |
            Qt::AlignVCenter,
            formatValue(value));
    }

    // Draw minor X-axis ticks.
    painter.setPen(
        QPen(
            QColor(
                120,
                120,
                120),
            1));

    const int minorCount =
        4;

    for (double value = firstXTick;
         value <= xMax + xStep * 0.01;
         value += xStep) {

        for (int i = 1;
             i < minorCount;
             ++i) {

            const double minorValue =
                value +
                xStep *
                    static_cast<double>(i) /
                    static_cast<double>(minorCount);

            if (minorValue >= xMax)
                continue;

            const double ratio =
                (minorValue - xMin) /
                xRange;

            const double x =
                plotRect.left() +
                ratio *
                    plotRect.width();

            painter.drawLine(
                QPointF(
                    x,
                    plotRect.bottom()),
                QPointF(
                    x,
                    plotRect.bottom() + 3));
        }
    }

    // Draw minor Y-axis ticks.
    for (double value = firstYTick;
         value <= yMax + yStep * 0.01;
         value += yStep) {

        for (int i = 1;
             i < minorCount;
             ++i) {

            const double minorValue =
                value +
                yStep *
                    static_cast<double>(i) /
                    static_cast<double>(minorCount);

            if (minorValue >= yMax)
                continue;

            const double ratio =
                (minorValue - yMin) /
                yRange;

            const double y =
                plotRect.bottom() -
                ratio *
                    plotRect.height();

            painter.drawLine(
                QPointF(
                    plotRect.left() - 3,
                    y),
                QPointF(
                    plotRect.left(),
                    y));
        }
    }

    painter.setRenderHint(
        QPainter::Antialiasing,
        true);
}

// Draws the data line.
void GraphWidget::drawData(
    QPainter& painter,
    const QRectF& plotRect,
    double xMin,
    double xMax,
    double yMin,
    double yMax)
{
    if (data_.isEmpty())
        return;

    const double xRange =
        xMax - xMin;

    const double yRange =
        yMax - yMin;

    QPainterPath path;

    bool started = false;

    for (const QPointF& point : data_) {

        if (!std::isfinite(point.x()) ||
            !std::isfinite(point.y())) {
            continue;
        }

        if (point.x() < xMin ||
            point.x() > xMax ||
            point.y() < yMin ||
            point.y() > yMax) {
            continue;
        }

        const double xRatio =
            (point.x() - xMin) /
            xRange;

        const double yRatio =
            (point.y() - yMin) /
            yRange;

        const QPointF pixel(
            plotRect.left() +
                xRatio *
                    plotRect.width(),

            plotRect.bottom() -
                yRatio *
                    plotRect.height());

        if (!started) {

            path.moveTo(pixel);

            started = true;
        }
        else {

            path.lineTo(pixel);
        }
    }

    if (!started)
        return;

    painter.setPen(
        QPen(
            QColor(
                25,
                100,
                200),
            2.0));

    painter.drawPath(path);
}

// Draws a vertical event marker and its label.
void GraphWidget::drawEventMarker(
    QPainter& painter,
    const QRectF& plotRect,
    double xMin,
    double xMax)
{
    if (event_markers_.isEmpty())
        return;

    const double xRange = xMax - xMin;

    if (xRange <= 0.0)
        return;

    painter.save();

    painter.setPen(
        QPen(
            Qt::red,
            2.0,
            Qt::DashLine));

    painter.setFont(
        QFont(
            "DejaVu Sans",
            8,
            QFont::DemiBold));

    for (const EventMarker& marker : event_markers_)
    {
        if (!std::isfinite(marker.x))
            continue;

        if (marker.x < xMin ||
            marker.x > xMax)
        {
            continue;
        }

        const double ratio =
            (marker.x - xMin) / xRange;

        const double x =
            plotRect.left() +
            ratio * plotRect.width();

        painter.drawLine(
            QPointF(
                x,
                plotRect.top()),
            QPointF(
                x,
                plotRect.bottom()));

    }

    painter.restore();
}

// Calculates a readable axis step.
double GraphWidget::niceStep(
    double range) const
{
    if (range <= 0.0)
        return 1.0;

    const double rough =
        range / 6.0;

    const double exponent =
        std::floor(
            std::log10(rough));

    const double fraction =
        rough /
        std::pow(
            10.0,
            exponent);

    double niceFraction = 1.0;

    if (fraction <= 1.0)
        niceFraction = 1.0;
    else if (fraction <= 2.0)
        niceFraction = 2.0;
    else if (fraction <= 5.0)
        niceFraction = 5.0;
    else
        niceFraction = 10.0;

    return niceFraction *
           std::pow(
               10.0,
               exponent);
}

// Formats an axis value.
QString GraphWidget::formatValue(
    double value) const
{
    const double absolute =
        std::abs(value);

    if (absolute >= 100.0)
        return QString::number(
            value,
            'f',
            0);

    if (absolute >= 10.0)
        return QString::number(
            value,
            'f',
            1);

    if (absolute >= 1.0)
        return QString::number(
            value,
            'f',
            2);

    return QString::number(
        value,
        'f',
        3);
}
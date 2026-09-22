#ifndef ROBOT_MOTION_TEST_APP_GRAPH_WIDGET_H
#define ROBOT_MOTION_TEST_APP_GRAPH_WIDGET_H

#include <QPointF>
#include <QString>
#include <QVector>
#include <QWidget>

class GraphWidget : public QWidget
{
    Q_OBJECT

public:

    explicit GraphWidget(
        const QString& title,
        const QString& xLabel,
        const QString& yLabel,
        QWidget* parent = nullptr);

    void setData(
        const QVector<QPointF>& data);

    void setEventMarker(
        double x,
        const QString& label);

    void clearEventMarker();

protected:

    void paintEvent(
        QPaintEvent* event) override;

private:

    void drawAxes(
        QPainter& painter,
        const QRectF& plotRect,
        double xMin,
        double xMax,
        double yMin,
        double yMax);

    void drawData(
        QPainter& painter,
        const QRectF& plotRect,
        double xMin,
        double xMax,
        double yMin,
        double yMax);

    void drawEventMarker(
        QPainter& painter,
        const QRectF& plotRect,
        double xMin,
        double xMax);

    double niceStep(
        double range) const;

    QString formatValue(
        double value) const;

    QString title_;
    QString x_label_;
    QString y_label_;

    struct EventMarker
    {
        double x;
        QString label;
    };

    QVector<QPointF> data_;
    QVector<EventMarker> event_markers_;
};

#endif

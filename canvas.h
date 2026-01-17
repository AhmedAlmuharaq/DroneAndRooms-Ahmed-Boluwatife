#ifndef CANVAS_H
#define CANVAS_H

#include <QWidget>
#include <QMouseEvent>
#include <QPaintEvent>
#include <serveranddrone.h>

class Canvas : public QWidget {
    Q_OBJECT
public:
    explicit Canvas(QWidget *parent = nullptr);
    ~Canvas() {
        for (auto &l:links) {
            delete l;
        }
    }
    void setWindow(const QPoint &origin, const QSize &size) {
        windowOrigin=origin;
        windowSize=size;
        windowScale={qreal(width())/windowSize.width(),qreal(height())/windowSize.height()};
    }
    QPoint getOrigin() { return windowOrigin; }
    QSize getSize() { return windowSize; }
    void paintEvent(QPaintEvent*) override;
    void resizeEvent(QResizeEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

    QList<Server> servers;
    QList<Drone> drones;
    QList<Link*> links;
    bool showGraph=false;
signals:

private:
    QPoint windowOrigin;
    QSize windowSize;
    QSizeF windowScale;
    qreal droneIconSize;
    QImage droneImg; ///< picture representing the drone in the canvas
};

#endif // CANVAS_H

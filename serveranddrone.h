#ifndef SERVERANDDRONE_H
#define SERVERANDDRONE_H
#include <QString>
#include <QPoint>
#include <QColor>
#include <QPainter>
#include <polygon.h>

const qreal accelation = 2.0; // unit/s²
const qreal speedMax = 1.0; // unit/s
const qreal speedLocal = 0.1; // unit/s
const qreal slowDownDistance = 20;
const qreal minDistance=5;
class Link;

class Server {
public :
    int id;
    QString name;
    QPointF position;
    QColor color;
    Polygon area;
    QList<Link*> links;
    QVector<QPair<Link*,qreal>> bestDistance;
};

class Link {
public:
    Link(Server *n1,Server *n2,const QPair<Vector2D,Vector2D> &edge);
    void draw(QPainter &painter);
    Server* getNode1() { return node1; }
    Server* getNode2() { return node2; }
    qreal getDistance() const { return distance; }
    Vector2D getEdgeCenter() { return Vector2D(edgeCenter.x(),edgeCenter.y()); }
private:
    Server *node1;
    Server *node2;
    QPointF edgeCenter;
    qreal distance;
};

class Drone {
public :
    QString name;
    Vector2D position;
    Server *target;
    qreal azimut=0;
    Vector2D destination;
    void move(qreal dt);
    Server* overflownArea(QList<Server>& list);
private:
    Server *connectedTo;
    Vector2D speed;
};

#endif // SERVERANDDRONE_H

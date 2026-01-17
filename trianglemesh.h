#ifndef TRIANGLEMESH_H
#define TRIANGLEMESH_H

#include <serveranddrone.h>
#include <polygon.h>

class TriangleMesh {
public:
    TriangleMesh(QList<Server> &servers);
    void setBox(const QPoint &origin,const QSize &size) { winX0=origin.x(); winY0=origin.y(); winX1=origin.x()+size.width(); winY1=origin.y()+size.height(); }
    QVector<Triangle>* getTriangles() { return &tabTriangles; }
    bool isInWindow(int x,int y) const { return (x>winX0 && x<winX1 && y>winY0 && y<winY1); }
    bool isInWindow(const Vector2D pos) const { return (pos.x>winX0 && pos.x<winX1 && pos.y>winY0 && pos.y<winY1); }
    int getWindowXmin() const { return winX0; }
    int getWindowYmin() const { return winY0; }
    int getWindowXmax() const { return winX1; }
    int getWindowYmax() const { return winY1; }
private:
    bool checkDelaunay();
    QVector<Vector2D> findOppositPointOfTrianglesWithCommonEdge(const Triangle &tri);
    QPair<Triangle*,Vector2D[4]> findOppositTriangle(Triangle *tri, Vector2D oppVertex);
    void flipTriangle(Triangle *);

    QVector<Vector2D> tabVertices;
    QVector<Triangle> tabTriangles;
    Polygon* convexHull=nullptr;
    int winX0,winX1,winY0,winY1;
};

#endif // TRIANGLEMESH_H

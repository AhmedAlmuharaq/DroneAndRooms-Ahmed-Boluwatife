#include "polygon.h"
#include <QDebug>
#include <QStack>

bool polarComparison(Vector2D P1,Vector2D P2) {
    double a1 = asin(P1.y/sqrt(P1.x*P1.x+P1.y*P1.y));
    if (P1.x<0.0) a1=M_PI-a1;
    double a2 = asin(P2.y/sqrt(P2.x*P2.x+P2.y*P2.y));
    if (P2.x<0.0) a2=M_PI-a2;
    return a1<a2;
}

Polygon::Polygon(QVector<Vector2D> &points) {
    assert(points.size()>3);
    auto p=points.begin();
    auto pymin=points.begin();

    // find point with minimal y and swap with first point
    while (p!=points.end()) {
        if (p->y<pymin->y) {
            pymin=p;
        }
        p++;
    }
    // swap
    if (pymin!=points.begin()) std::iter_swap(points.begin(), pymin);

    Vector2D origin(points.begin()->x,points.begin()->y);
    // copy points in a set of points relative to points[0]
    QVector<Vector2D> pointsRelative;
    for (auto pOrig:points) {
        pointsRelative.push_back(Vector2D(pOrig.x-origin.x,pOrig.y-origin.y));
    }

    // sorting point with angular criteria
    std::sort(pointsRelative.begin()+1,
              pointsRelative.end(),polarComparison);

    QStack<Vector2D*> CHstack;
    Vector2D *top_1,*top;
    CHstack.push(&pointsRelative[0]);
    CHstack.push(&pointsRelative[1]);
    CHstack.push(&pointsRelative[2]);

    auto pi=pointsRelative.begin()+3;
    while (pi!=pointsRelative.end()) {
        top = CHstack.top();
        CHstack.pop();
        top_1 = CHstack.top();
        CHstack.push(top);
        while (!isOnTheLeft(&(*pi),top_1,top)) {
            CHstack.pop();
            // update values of top and top_1
            top = CHstack.top();
            CHstack.pop();
            top_1 = CHstack.top();
            CHstack.push(top);
        }
        CHstack.push(&(*pi));
        pi++;
    }

    // get stack points to create current polygon
    while (!CHstack.empty()) {
        tabPts.push_front(*(CHstack.top())+origin);
        CHstack.pop();
    }
    tabPts.push_back(tabPts[0]);// polygon propriety (N+1 vertices with P_N=P_0)
    triangulate();
}

void Polygon::addVertex(float x, float y) {
    if (tabPts.empty()) {
        tabPts.push_back(Vector2D(x,y));
        tabPts.push_back(Vector2D(x,y));
    } else {
        auto N=tabPts.size()-1;
        tabPts[N].x=x;
        tabPts[N].y=y;
///< the last vertex duplicates the first one
        tabPts.push_back(tabPts[0]);
    }
}

const float doorWidth=20.0;
void Polygon::draw(QPainter &painter) const {
    if (tabPts.empty()) return;

    QPen pen(Qt::black);
    pen.setWidth(3);
    ///< use the drawPolygon method of QPainter
    auto N=tabPts.size();
    QPoint *points=new QPoint[N];
    for (int i=0; i<N; i++) {
        points[i].setX(tabPts[i].x);
        points[i].setY(tabPts[i].y);
    }
    painter.setPen(pen);

    painter.drawPolygon(points,N,Qt::OddEvenFill);
    delete [] points;

    /***********************************************************************
     * Exercise 1 — Doors (Geometry)
     *
     * Goal:
     *   Draw a "door" of fixed width (doorWidth) in the middle of each polygon edge.
     *
     * Geometry:
     *   Given an edge segment AB:
     *     M  = (A + B) / 2                 (midpoint)
     *     u  = (B - A) / ||B - A||         (unit direction along the edge)
     *     Q0 = M - u * (doorWidth/2)       (door endpoint 1)
     *     Q1 = M + u * (doorWidth/2)       (door endpoint 2)
     *
     * Drawing:
     *   Draw segment Q0Q1 using a white pen with width 7.
     ***********************************************************************/
    {
        QPen doorPen(Qt::white);
        doorPen.setWidth(7);
        painter.setPen(doorPen);

        // tabPts stores (N+1) points where last == first
        const int nv = nbVertices(); // number of edges
        for (int i = 0; i < nv; ++i) {
            const Vector2D A = tabPts[i];
            const Vector2D B = tabPts[i + 1];

            // Edge direction vector
            const Vector2D AB = B - A;
            const double len = AB.length();
            if (len < 1e-9) continue; // safety against degenerate edges

            // Unit direction and midpoint
            const Vector2D u = (1.0 / len) * AB;
            const Vector2D M = 0.5 * (A + B);

            // Door endpoints
            const Vector2D half = (doorWidth * 0.5) * u;
            const Vector2D Q0 = M - half;
            const Vector2D Q1 = M + half;

            painter.drawLine(QPointF(Q0.x, Q0.y), QPointF(Q1.x, Q1.y));
        }
    }



}

QPair<Vector2D,Vector2D> Polygon::getBoundingBox() const {
    Vector2D min=tabPts[0],max=tabPts[0];
    for (auto &pt:tabPts) {
        if (pt.x<min.x) min.x = pt.x;
        if (pt.y<min.y) min.y = pt.y;
        if (pt.x>max.x) max.x = pt.x;
        if (pt.y>max.y) max.y = pt.y;
    }
    return QPair<Vector2D,Vector2D>(min,max);
}

void Polygon::triangulate() {
    /// @todo write the code
    /// 1. Copy the poly polygon in a temporary version (tmp)
    Polygon tmp(*this);
    QList<Vector2D*> pointListPtr;

    /// 2. search a first consecutive group of three vertices that check:
    /// - CCW oriented
    /// - does not contain any other vertex
    int i=0;
    auto N=tmp.nbVertices();
    while (N>=3) {
        Triangle t(tmp[i%N],tmp[(i+1)%N],tmp[(i+2)%N]);
        // fill the list (pointListPtr) with all the vertices of the polygon
        // but the vertices of the triangle.
        pointListPtr.clear();
        for (int j=0; j<N; j++) {
            if (j!=i && j!=(i+1)%N && j!=(i+2)%N) {
                pointListPtr.push_back(&(tmp.tabPts[j]));
            }
        }

        if (t.isCCW() && !t.containsOneOf(pointListPtr)) {
            /// 3. add the triangle in the list
            triangles.push_back(t);
            /// 4. remove middle vertex from the tmp polygon
            tmp.remove((i+1)%N);
            N--;
        } else {
            i=(i+1)%N;
        }
    }
}

void Polygon::clip(int x0,int y0,int x1,int y1) {
    // cut point out of the zone
    int i;
    for (int k=0; k<4; k++) {
        // find a point outside
        i=0;
        int N=nbVertices();
        while (i<N && tabPts[i].x>=x0 && tabPts[i].x<=x1 && tabPts[i].y>=y0 && tabPts[i].y<=y1) {
            i++;
        }
        if (i!=N) {
            Vector2D P[2];
            int i_1=(i==0)?N:i-1;
            if (tabPts[i].y>y1) {
                int n=0;
                if (tabPts[i_1].y<y1) {
                    P[n++].set(tabPts[i_1].x+(tabPts[i].x-tabPts[i_1].x)*((y1-tabPts[i_1].y)/(tabPts[i].y-tabPts[i_1].y)),y1);
                }
                if (tabPts[i+1].y<y1) {
                    P[n++].set(tabPts[i].x+(tabPts[i+1].x-tabPts[i].x)*((y1-tabPts[i].y)/(tabPts[i+1].y-tabPts[i].y)),y1);
                }
                if (n==2) insertPoint(P[1],i+1);
                if (n>0) {
                    tabPts[i]=P[0];
                    if (i==0) tabPts[N]=P[0];
                }
            } else if (tabPts[i].y<y0) {
                int n=0;
                if (tabPts[i_1].y>y0) {
                    P[n++].set(tabPts[i_1].x+(tabPts[i].x-tabPts[i_1].x)*((y0-tabPts[i_1].y)/(tabPts[i].y-tabPts[i_1].y)),y0);
                }
                if (tabPts[i+1].y>y0) {
                    P[n++].set(tabPts[i].x+(tabPts[i+1].x-tabPts[i].x)*((y0-tabPts[i].y)/(tabPts[i+1].y-tabPts[i].y)),y0);
                }
                if (n==2) insertPoint(P[1],i+1);
                if (n>0) {
                    tabPts[i]=P[0];
                    if (i==0) tabPts[N]=P[0];
                }
            } else if (tabPts[i].x>x1) {
                int n=0;
                if (tabPts[i_1].x<x1) {
                    P[n++].set(x1,tabPts[i_1].y+(tabPts[i].y-tabPts[i_1].y)*((x1-tabPts[i_1].x)/(tabPts[i].x-tabPts[i_1].x)));
                }
                if (tabPts[i+1].x<x1) {
                    P[n++].set(x1,tabPts[i].y+(tabPts[i+1].y-tabPts[i].y)*((x1-tabPts[i].x)/(tabPts[i+1].x-tabPts[i].x)));
                }
                if (n==2) insertPoint(P[1],i+1);
                if (n>0) {
                    tabPts[i]=P[0];
                    if (i==0) tabPts[N]=P[0];
                }
            } else if (tabPts[i].x<x0) {
                int n=0;
                if (tabPts[i_1].x>x0) {
                    P[n++].set(x0,tabPts[i_1].y+(tabPts[i].y-tabPts[i_1].y)*((x0-tabPts[i_1].x)/(tabPts[i].x-tabPts[i_1].x)));
                }
                if (tabPts[i+1].x>x0) {
                    P[n++].set(x0,tabPts[i].y+(tabPts[i+1].y-tabPts[i].y)*((x0-tabPts[i].x)/(tabPts[i+1].x-tabPts[i].x)));
                }
                if (n==2) insertPoint(P[1],i+1);
                if (n>0) {
                    tabPts[i]=P[0];
                    if (i==0) tabPts[N]=P[0];
                }
            }
        }
    }


    i=0;
    while (i<nbVertices()) {
        // case Left/Bottom
        if (tabPts[i].x==x0 && tabPts[i+1].y==y0) {
            insertPoint(Vector2D(x0,y0),i+1);
            i++;
        } else // case Bottom/Right
            if (tabPts[i].y==y0 && tabPts[i+1].x==x1) {
                insertPoint(Vector2D(x1,y0),i+1);
                i++;
            } else // case Right/Top
                if (tabPts[i].x==x1 && tabPts[i+1].y==y1) {
                    insertPoint(Vector2D(x1,y1),i+1);
                    i++;
                } else // case Top/Left
                    if (tabPts[i].y==y1 && tabPts[i+1].x==x0) {
                        insertPoint(Vector2D(x0,y1),i+1);
                        i++;
                    }

        i++;
    }
}

void Triangle::computeCircle() {
    Vector2D AB = tabPts[1]-tabPts[0];
    Vector2D AC=tabPts[2]-tabPts[0];
    //OBprim = OA+0.5 AC
    Vector2D OBprim = tabPts[0] + 0.5*AC;
    Vector2D VAC = AC.orthoNormed();

    double k=(AB*AB - AC*AB)/(2* (VAC*AB));

    circumCenter = OBprim+ k * VAC;
    circumRadius = (circumCenter-tabPts[0]).length();
}

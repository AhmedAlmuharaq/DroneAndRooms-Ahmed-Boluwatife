/**
 * @author Benoît Piranda
 * @date 7/11/2024
 * @todo Make more tests of the triangulation algorithm with complex shapes.
 **/

#ifndef POLYGON_H
#define POLYGON_H
#include "vector2d.h"
#include <determinant.h>
#include <QPainter>
#include <QDebug>

/**
 * @brief The Triangle class stores 3 pointers to existing Vector2D vertices.
 * It is used by the Polygon class to create a set of internal triangles.
 */
class Triangle {
    Vector2D tabPts[3]; ///< array of 3 pointers to vertices
    bool isDelaunay=false; ///< is drawn as Delaunay triangle
    bool isFlippable=false; ///< is drawn as Delaunay but not flippable triangle
    Vector2D circumCenter; ///< the center of the triangle (by computeCenter)
    float circumRadius; ///< the radius of the circumCircle (by computeCenter)
public:
    /**
     * @brief Constuctor of triangle with pointers to vertices,
     * @warning The order of vertices must be CCW
     * @param p_p0 P0
     * @param p_p1 P1
     * @param p_p2 P2
     */
    Triangle(const Vector2D &p_p0,const Vector2D &p_p1,const Vector2D &p_p2) {
        tabPts[0]=p_p0;
        tabPts[1]=p_p1;
        tabPts[2]=p_p2;
        computeCircle();
    }
    void update(const Vector2D &p_p0,const Vector2D &p_p1,const Vector2D &p_p2) {
        tabPts[0]=p_p0;
        tabPts[1]=p_p1;
        tabPts[2]=p_p2;
        computeCircle();
    }
    void computeCircle();
    /**
     * @brief operator [] to get vertex #i coordinates
     * @param i
     * @return vertex #i coordinates.
     */
    Vector2D operator[](int i) const { return tabPts[i]; }
    /**
     * @brief isCCW: equivalent to check if the third point
     * is on the left of the first edge
     * @return true if the triangle is Counterclock Wise oriented
     */
    bool isCCW() {
        Vector2D AB = (tabPts[1])-(tabPts[0]);
        Vector2D AP = (tabPts[2])-(tabPts[0]);
        // z component of AB x AP
        return (AB.x*AP.y-AB.y*AP.x)>0;
    }
    /**
     * @brief contains
     * @param lst
     * @return true if one or more position of lst is inside the triangle
     */
    bool containsOneOf(const QList<Vector2D*> &lst) {
        auto it=lst.begin();
        while (it!=lst.end() && !contains(*it)) {
            it++;
        }
        return (it!=lst.end());
    }

    bool contains(const Vector2D &ptrPt) const {
        return isOnTheLeft(ptrPt,0) &&
               isOnTheLeft(ptrPt,1) &&
               isOnTheLeft(ptrPt,2);
    }
    /**
     * @brief isOnTheLeft
     * @param p tested point
     * @param i the number of the edge (P_iP_{i+1})
     * @return true if p is on the left of the edge P_iP_{i+1}
     */
    bool isOnTheLeft(const Vector2D &p, int i) const {
        Vector2D AB = (tabPts[(i+1)%3])-(tabPts[i]);
        Vector2D AP = p-(tabPts[i]);
        // z component of AB x AP
        return (AB.x*AP.y-AB.y*AP.x)>=0;
    }
    void print() {
        qDebug() << tabPts[0].x << "," << tabPts[0].y << "/"
         << tabPts[1].x << "," << tabPts[1].y << "/"
                 << tabPts[2].x << "," << tabPts[2].y;
    }
    /**
     * @brief area
     * @warning the area is in u square. (u=10 pixels).
     * @return the surface of the triangle
     */
    double area() const {
        Vector2D AB=(tabPts[1])-(tabPts[0]);
        Vector2D AC=(tabPts[2])-(tabPts[0]);
        return 0.005*(AB.x*AC.y-AB.y*AC.x); // convertion to u² unit
    }

    inline void setDelaunay(bool d, bool f) {
        isDelaunay=d;
        isFlippable=f;
    }
    bool canBeFlipped() {
        return (!isDelaunay && isFlippable);
    }
    bool checkDelaunay(const QVector<Vector2D> &tabVertices) {
        auto it=tabVertices.begin();
        while (it!=tabVertices.end() && circleContains(*it)) {
            it++;
        }
        isDelaunay = it==tabVertices.end();
        isFlippable=false;
        return isDelaunay;
    }
    Vector2D getNextVertex(const Vector2D &pt) const {
        if (pt==tabPts[0]) return tabPts[1];
        if (pt==tabPts[1]) return tabPts[2];
        if (pt==tabPts[2]) return tabPts[0];
        return Vector2D(0,0);
    }

    Vector2D getPrevVertex(const Vector2D &pt) const {
        if (pt==tabPts[0]) return tabPts[2];
        if (pt==tabPts[1]) return tabPts[0];
        if (pt==tabPts[2]) return tabPts[1];
        return Vector2D(0,0);
    }
    bool hasEdge(const Vector2D& A,const Vector2D& B) const {
        return (A==tabPts[0] && B==tabPts[1])||
               (A==tabPts[1] && B==tabPts[2])||
               (A==tabPts[2] && B==tabPts[0]);
    }
    bool hasVertex(const Vector2D &M) const {
        return (M==tabPts[0])||(M==tabPts[1])||(M==tabPts[2]);
    }
    bool operator==(const Triangle &other) const {
        return (tabPts[0]==other.tabPts[0] || tabPts[0]==other.tabPts[1] || tabPts[0]==other.tabPts[2]) &&
               (tabPts[1]==other.tabPts[0] || tabPts[1]==other.tabPts[1] || tabPts[1]==other.tabPts[2]) &&
               (tabPts[2]==other.tabPts[0] || tabPts[2]==other.tabPts[1] || tabPts[2]==other.tabPts[2]);
    }
    bool circleContains(const Vector2D&M) {
        Matrix33 mat;
        Vector2D A = tabPts[0];
        Vector2D B = tabPts[1];
        Vector2D C = tabPts[2];
        mat.m[0][0] = (A.x - M.x);
        mat.m[0][1] = (A.y - M.y);
        mat.m[0][2] = (A.x*A.x-M.x*M.x)+(A.y*A.y-M.y*M.y);
        mat.m[1][0] = (B.x - M.x);
        mat.m[1][1] = (B.y - M.y);
        mat.m[1][2] = (B.x*B.x-M.x*M.x)+(B.y*B.y-M.y*M.y);
        mat.m[2][0] = (C.x - M.x);
        mat.m[2][1] = (C.y - M.y);
        mat.m[2][2] = (C.x*C.x-M.x*M.x)+(C.y*C.y-M.y*M.y);
        return mat.determinant()<=0; // positive for outside points and equal for A,B,C
    }
    Vector2D getCenter() const {
        return circumCenter;
    }
    Vector2D getEdgeTo(const Vector2D &p) const {
        if (tabPts[0]==p) return tabPts[2];
        if (tabPts[1]==p) return tabPts[0];
        if (tabPts[2]==p) return tabPts[1];
        return nullptr;
    }

    Vector2D getEdgeFrom(const Vector2D &p) const {
        if (tabPts[0]==p) return tabPts[1];
        if (tabPts[1]==p) return tabPts[2];
        if (tabPts[2]==p) return tabPts[0];
        return nullptr;
    }

    Vector2D nextEdgeNormal(const Vector2D pt) const {
        if (pt==tabPts[0]) return Vector2D(tabPts[1].y-tabPts[0].y,-(tabPts[1].x-tabPts[0].x));
        if (pt==tabPts[1]) return Vector2D(tabPts[2].y-tabPts[1].y,-(tabPts[2].x-tabPts[1].x));
        if (pt==tabPts[2]) return Vector2D(tabPts[0].y-tabPts[2].y,-(tabPts[0].x-tabPts[2].x));
        return Vector2D(0,0);
    }

    Vector2D previousEdgeNormal(const Vector2D pt) const {
        if (pt==tabPts[0]) return Vector2D(-(tabPts[2].y-tabPts[0].y),tabPts[2].x-tabPts[0].x);
        if (pt==tabPts[1]) return Vector2D(-(tabPts[0].y-tabPts[1].y),tabPts[0].x-tabPts[1].x);
        if (pt==tabPts[2]) return Vector2D(-(tabPts[1].y-tabPts[2].y),tabPts[1].x-tabPts[2].x);
        return Vector2D(0,0);
    }

};

/**
 *  @brief Polygon class allow to create, draw and manipulate
 *  polygons, especially check if a point is inside and compute
 *  the surface of the polygon.
*/
class Polygon {
private:
    ///< @warning Store N+1 vertices in tabPts array, first is duplicated in last.
    QVector<Vector2D> tabPts; ///< array of vertex positions
    QVector<Triangle> triangles; ///< array of triangles for the triangulation process
public:
    /**
     * @brief Constructor of a polygon.
     */
    Polygon(QVector<Vector2D> &points);
    Polygon() {}
    void remove(int i) {
        assert(i>=0 && i<tabPts.size()-1);
        tabPts.removeAt(i);
        tabPts[tabPts.size()-1]=tabPts[0];
    }

    /**
     * @brief nbVertices
     * @return the number of vertices of the polygon
     */
    int nbVertices() const { return tabPts.size()==0?0:tabPts.size()-1; }
    /**
     * @brief Add a new vertex at the end of the list
     * @warning The number of vertices added to the polygon must be lower than Nmax.
     * @param x abscissa of the new vertex
     * @param y ordinate of the new vertex
     */
    void addVertex(float x,float y);
    void addVertex(const Vector2D &v) {
        addVertex(v.x,v.y);
    }
    QPair<Vector2D,Vector2D> getEdge(int i) {
        i=i%nbVertices();
        return {tabPts[i],tabPts[i+1]};
    }
    /**
     * @brief return the boundig box coordinates of the polygon
     * @return The first vector is the left bottom corner and the second vector is the right top corner of the box.
     */
    QPair<Vector2D,Vector2D> getBoundingBox() const;
    /**
     * @brief A way to read the list of vertices of the polygon.
     * @warning The returned array is read-only!
     * @param n number of vertices in the array.
     * @return a read-only version of the array of vertices.
     */
    Vector2D operator[](int i) const { return tabPts[i]; }
    /**
     * @brief Draw the polygon.
     * @param painter Current painter context.
     * @param showWall shows the internal walls.
     * @param thickness of the walls.
     */
    void draw(QPainter &painter) const;
    /**
     * @brief triangulate the polygon and store triangles in "triangles" array.
     */
    void triangulate();

    /**
     * @brief isOnTheLeft
     * @param p tested point
     * @param i the number of the edge (P_iP_{i+1})
     * @return true if p is on the left of the edge P_iP_{i+1}
     */
    bool isOnTheLeft(const Vector2D &p, int i) const {
        Vector2D AB = tabPts[i+1]-tabPts[i];
        Vector2D AP = p-tabPts[i];
        // z component of AB x AP
        return (AB.x*AP.y-AB.y*AP.x)>=0;
    }
    /**
     * @brief isOnTheLeft
     * @param P tested point
     * @param A
     * @param B
     * @return true if Ap is on the left of [AB]
     */
    bool isOnTheLeft(const Vector2D *p,const Vector2D *A,const Vector2D *B) {
        Vector2D AB = *B-*A,AP = *p-*A;
        return (AB.x*AP.y - AB.y*AP.x)>=0;
    }
    /**
     * @brief isConvex
     * @return true if the polygon is convex
     */
    bool isConvex() const {
        int N=nbVertices();
        if (N<3) return false;
        int i=0;
        while (i<N && isOnTheLeft(tabPts[(i+2)%N],i)) {
            i++;
        }
        return i>=N;
    }
    bool isAVertex(const Vector2D &v) {
        auto itV=tabPts.begin();
        while (itV!=tabPts.end() && !(*itV==v)) {
            itV++;
        }
        return (itV!=tabPts.end());
    }
    QVector<Triangle> getTriangles() {
        return triangles;
    }
    /**
     * @brief area
     * @return the surface of a polygon
     */
    double area() const {
        /// @todo write the code
        double res=0;
        for (auto &t:triangles) {
                res+=t.area();
        }
        return res;
    }
    void clip(int x0,int y0,int x1,int y1);
    void insertPoint(const Vector2D &p,int index) {
        tabPts.insert(index,p);
        tabPts[tabPts.size()-1]=tabPts[0];
    }
    bool contains(const Vector2D& pt) {
        auto t = triangles.begin();
        while (t!=triangles.end() && !t->contains(pt)) {
            t++;
        }
        return t!=triangles.end();
    }
};

#endif // POLYGON_H

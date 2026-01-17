#include <trianglemesh.h>

TriangleMesh::TriangleMesh(QList<Server> &servers) {
    // fill tabVerticies from servers
    for (auto &s:servers) {
        tabVertices.push_back(Vector2D(s.position.x(),s.position.y()));
    }
    // create the convex hull
    Polygon convexHull(tabVertices);

    tabTriangles=convexHull.getTriangles();
    // list of server that are not in the convexhull
    QList<Vector2D> internalVertices;
    for (auto &s:servers) {
        Vector2D p(Vector2D(s.position.x(),s.position.y()));
        if (!convexHull.isAVertex(p)) {
            internalVertices.append(p);
        }
    }

    for (auto &vertex:internalVertices) {
        auto tri=tabTriangles.begin();
        while (tri!=tabTriangles.end() && !tri->contains(&vertex)) tri++;
        if (tri!=tabTriangles.end()) {
            Vector2D v0 = (*tri)[0];
            Vector2D v1 = (*tri)[1];
            Vector2D v2 = (*tri)[2];
            tri->update(v0,v1,vertex);
            tabTriangles.push_back(Triangle(v1,v2,vertex));
            tabTriangles.push_back(Triangle(v2,v0,vertex));
        }
    }

    while (!checkDelaunay()) {
        // search the first triangle that is not delaunay compliant and flippabe
        auto it = tabTriangles.begin();
        while (it!=tabTriangles.end() && !(*it).canBeFlipped()) {
            it++;
        }
        if (it!=tabTriangles.end()) {
            flipTriangle(&(*it));
        }
    }

}

bool TriangleMesh::checkDelaunay() {
    bool areAllDelaunay=true;
    for (auto &tri:tabTriangles) {
        bool res = tri.checkDelaunay(tabVertices);
        if (!res) {
            auto L=findOppositPointOfTrianglesWithCommonEdge(tri);
            auto it=L.begin();
            while (it!=L.end() && tri.circleContains(*it)) {
                it++;
            }
            tri.setDelaunay(false,it!=L.end());
        }
        areAllDelaunay=areAllDelaunay && res;
    }
    return areAllDelaunay;
}

void TriangleMesh::flipTriangle(Triangle *ptrTriangleClicked) {
    // get the list of opposit points (0..3)
    auto L=findOppositPointOfTrianglesWithCommonEdge(*ptrTriangleClicked);
    // search the point that is inside the circumcircle
    auto it=L.begin();
    while (it!=L.end() && ptrTriangleClicked->circleContains(*it)) {
        it++;
    }
    // if it exists
    if (it!=L.end()) {
        // search the opposit triangle and the ordered list of 4 points
        auto res = findOppositTriangle(ptrTriangleClicked,*it);
        // switch the vertices
        ptrTriangleClicked->update(res.second[0],res.second[1],res.second[3]);
        res.first->update(res.second[1],res.second[2],res.second[3]);
    }
}

QPair<Triangle*,Vector2D[4]> TriangleMesh::findOppositTriangle(Triangle *tri, Vector2D oppVertex) {
    QPair<Triangle*,Vector2D[4]> res;

    // search a triangle with (P0 oppVertex P1) vertices
    auto it=tabTriangles.begin();
    Triangle ref((*tri)[0],oppVertex,(*tri)[1]);
    while (it!=tabTriangles.end() && !((*it)==ref)) {
        it++;
    }
    if (it!=tabTriangles.end()) {
        res.first = &(*it);
        res.second[0]=(*tri)[0];
        res.second[1]=oppVertex;
        res.second[2]=(*tri)[1];
        res.second[3]=(*tri)[2];
        return res;
    }
    // search a triangle with (P1 oppVertex P2) vertices
    it=tabTriangles.begin();
    ref = Triangle((*tri)[1],oppVertex,(*tri)[2]);
    while (it!=tabTriangles.end() && !((*it)==ref)) {
        it++;
    }
    if (it!=tabTriangles.end()) {
        res.first = &(*it);
        res.second[0]=(*tri)[1];
        res.second[1]=oppVertex;
        res.second[2]=(*tri)[2];
        res.second[3]=(*tri)[0];
        return res;
    }
    // search a triangle with (P2 oppVertex P0) vertices
    it=tabTriangles.begin();
    ref = Triangle((*tri)[2],oppVertex,(*tri)[0]);
    while (it!=tabTriangles.end() && !((*it)==ref)) {
        it++;
    }
    if (it!=tabTriangles.end()) {
        res.first = &(*it);
        res.second[0]=(*tri)[2];
        res.second[1]=oppVertex;
        res.second[2]=(*tri)[0];
        res.second[3]=(*tri)[1];
        return res;
    }
    return res;
}

QVector<Vector2D> TriangleMesh::findOppositPointOfTrianglesWithCommonEdge(const Triangle &tri) {
    QVector<Vector2D> res;
    for (auto &t:tabTriangles) {
        if (tri.hasEdge(t[1],t[0])) res.push_back(t[2]);
        else if (tri.hasEdge(t[2],t[1])) res.push_back(t[0]);
        else if (tri.hasEdge(t[0],t[2])) res.push_back(t[1]);
    }
    return res;
}

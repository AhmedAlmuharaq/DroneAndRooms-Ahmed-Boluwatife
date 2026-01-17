#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <canvas.h>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <trianglemesh.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // load initial simple case
    loadJson("../../../json/simple.json");
}

MainWindow::~MainWindow()
{
    delete ui;
}

bool MainWindow::loadJson(const QString& title) {
    QFile file(title);
    // --- RESET previous case (VERY IMPORTANT) ---
    ui->canvas->servers.clear();
    ui->canvas->drones.clear();

    // If you store Link* pointers, delete them to avoid leaks
    qDeleteAll(ui->canvas->links);
    ui->canvas->links.clear();

    // Also clear any other cached stuff you keep (optional)
    ui->canvas->repaint();

    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Impossible d'ouvrir le fichier:" << title;
        return false;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(data, &error);
    if (error.error != QJsonParseError::NoError) {
        qWarning() << "Erreur JSON:" << error.errorString();
        return false;
    }
    if (!doc.isObject()) {
        qWarning() << "Le document JSON n'est pas un objet.";
        return false;
    }

    QJsonObject root = doc.object();

    // --- Window ---
    if (root.contains("window") && root["window"].isObject()) {
        QJsonObject win = root["window"].toObject();

        auto origin = win.value("origine").toString().split(",");
        auto size   = win.value("size").toString().split(",");
        QPoint wOrigin={origin[0].toInt(),origin[1].toInt()};
        QSize wSize={size[0].toInt(),size[1].toInt()};
        qDebug() << "Window.origine =" << wOrigin;
        qDebug() << "Window.size    =" << wSize;
        ui->canvas->setWindow(wOrigin,wSize);
    }

    // --- Servers ---
    if (root.contains("servers") && root["servers"].isArray()) {
        int num=0;
        QJsonArray arr = root["servers"].toArray();
        for (const QJsonValue &v : arr) {
            if (!v.isObject()) continue;
            QJsonObject obj = v.toObject();
            Server s;
            s.name = obj.value("name").toString();
            QString pos = obj.value("position").toString();
            auto parts = pos.split(',');
            if (parts.size() == 2)
                s.position = QPoint(parts[0].toInt(), parts[1].toInt());
            s.color = QColor(obj.value("color").toString());
            s.id=num++;
            ui->canvas->servers.append(s);
            qDebug() << "Server:" << s.id << "," << s.name << s.position << s.color;
        }
    }

    // --- Drones ---
    if (root.contains("drones") && root["drones"].isArray()) {
        QJsonArray arr = root["drones"].toArray();

        for (const QJsonValue &v : arr) {
            if (!v.isObject()) continue;
            QJsonObject obj = v.toObject();
           Drone d;
            d.name = obj.value("name").toString();
            QString pos = obj.value("position").toString();
            auto parts = pos.split(',');
            if (parts.size() == 2)
                d.position = Vector2D(parts[0].toInt(), parts[1].toInt());
            QString name = obj.value("target").toString();
            // search name in server list
            d.target=nullptr;
            auto it=ui->canvas->servers.begin();
            while (it!=ui->canvas->servers.end() && it->name!=name) it++;
            if (it!=ui->canvas->servers.end()) {
                d.target=&(*it);
                qDebug() << "Drone:" << d.name << "(" << d.position.x << "," << d.position.y << ") →" << d.target->name;
            } else {
                qDebug() << "error in JsonFile: bad destination name: " << name;
            }
            ui->canvas->drones.append(d);
        }
    }

    createVoronoiMap();
    createServersLinks();
    fillDistanceArray();
    return true;
}

void MainWindow::createVoronoiMap() {
    TriangleMesh mesh(ui->canvas->servers);
    mesh.setBox(ui->canvas->getOrigin(),ui->canvas->getSize());

    auto triangles = mesh.getTriangles();
    auto m_servor = ui->canvas->servers.begin();
    QVector<const Triangle*> tabTri;
    while (m_servor!=ui->canvas->servers.end()) {
        // for all vertices of the mesh
        const Vector2D vert((*m_servor).position.x(),(*m_servor).position.y());
        auto mt_it = triangles->begin();
        tabTri.clear(); // tabTri: list of triangles containing m_vert
        while (mt_it!=triangles->end()) {
            if ((*mt_it).hasVertex(vert)) {
                tabTri.push_back(&(*mt_it));
            }
            mt_it++;
        }
        // find left border
        auto first = tabTri.begin();
        auto tt_it = tabTri.begin();
        bool found=false;
        while (tt_it!=tabTri.end() && !found) {
            auto comp_it = tabTri.begin();
            while (comp_it!=tabTri.end() && (*tt_it)->getNextVertex(vert)!=(*comp_it)->getPrevVertex(vert)) {
                comp_it++;
            }
            if (comp_it==tabTri.end()) {
                first=tt_it;
                found=true;
            }
            tt_it++;
        }
        // create polygon

        //poly->setColor((*m_servor)->color);
        tt_it=first;
        if (found && mesh.isInWindow((*tt_it)->getCenter().x,(*tt_it)->getCenter().y)) { // add a point for the left border
            Vector2D V = (*first)->nextEdgeNormal(vert);
            float k;
            if (V.x > 0) { // (circumCenter+k V).x=width
                k = (mesh.getWindowXmax() - (*first)->getCenter().x) / float(V.x);
            } else {
                k = (mesh.getWindowXmin()-(*first)->getCenter().x) / float(V.x);
            }
            if (V.y > 0) { // (circumCenter+k V).y=height
                k = fmin(k, (mesh.getWindowYmax() - (*first)->getCenter().y) / float(V.y));
            } else {
                k = fmin(k, (mesh.getWindowYmin()-(*first)->getCenter().y) / float(V.y));
            }
            m_servor->area.addVertex(Vector2D((*first)->getCenter() + k * V));
            Vector2D pt = (*first)->getCenter() + k * V;
        }
        auto comp_it = first;
        do {
            m_servor->area.addVertex((*tt_it)->getCenter());
            // search triangle on right of tt_it
            comp_it = tabTri.begin();
            while (comp_it!=tabTri.end() && (*tt_it)->getPrevVertex(vert)!=(*comp_it)->getNextVertex(vert)) {
                comp_it++;
            }
            if (comp_it!=tabTri.end()) tt_it = comp_it;
        } while (tt_it!=first && comp_it!=tabTri.end());
        if (found && mesh.isInWindow((*tt_it)->getCenter())) { // add a point for the right border
            Vector2D V = (*tt_it)->previousEdgeNormal(vert);
            float k;
            if (V.x > 0) { // (circumCenter+k V).x=width
                k = (mesh.getWindowXmax() - (*tt_it)->getCenter().x) / float(V.x);
            } else {
                k = (mesh.getWindowXmin()-(*tt_it)->getCenter().x) / float(V.x);
            }
            if (V.y > 0) { // (circumCenter+k V).y=height
                k = fmin(k, (mesh.getWindowYmax() - (*tt_it)->getCenter().y) / float(V.y));
            } else {
                k = fmin(k, (mesh.getWindowYmin()-(*tt_it)->getCenter().y) / float(V.y));
            }
            m_servor->area.addVertex(Vector2D((*tt_it)->getCenter() + k * V));
            Vector2D pt = (*tt_it)->getCenter() + k * V;
        }
        m_servor->area.clip(mesh.getWindowXmin(),mesh.getWindowYmin(),mesh.getWindowXmax(),mesh.getWindowYmax());
        m_servor->area.triangulate();

        m_servor++;
    }
}

void MainWindow::createServersLinks()
{
    /***********************************************************************
     ******************
     ******************
     * Exercise 2 — Graph Creation (Neighbors)
     *
     * Goal:
     *   Connect servers with a Link if their Voronoi polygons share a common edge.
     *
     * Neighbor criterion:
     *   Two servers i and j are neighbors if polygon(i) and polygon(j)
     *   contain an identical edge segment (same endpoints, possibly reversed).
     *
     * Output:
     *   - ui->canvas->links contains all created Link*
     *   - each Server.links adjacency list is filled
     *
     * Complexity:
     *   For n servers and ~v edges per polygon:
     *     O(n^2 * v^2) edge comparisons (acceptable for small n).
     ***********************************************************************/

    // Clean existing links (avoid duplicates and memory leaks if reloading JSON)
    for (auto *l : ui->canvas->links) delete l;
    ui->canvas->links.clear();

    // Clear adjacency lists
    for (auto &s : ui->canvas->servers) s.links.clear();

    // Small epsilon-based comparison for points (floating geometry)
    auto samePoint = [](const Vector2D &a, const Vector2D &b) -> bool {
        const double eps2 = 1e-6;
        return a.distance2(b) <= eps2;
    };

    const int n = ui->canvas->servers.size();

    // Compare each pair of servers only once (i < j)
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {

            Polygon &polyA = ui->canvas->servers[i].area;
            Polygon &polyB = ui->canvas->servers[j].area;

            bool foundCommonEdge = false;
            QPair<Vector2D, Vector2D> commonEdge;

            // Compare edges of polygon A with edges of polygon B
            for (int ea = 0; ea < polyA.nbVertices() && !foundCommonEdge; ++ea) {
                const auto eA = polyA.getEdge(ea); // (P_k, P_{k+1})

                for (int eb = 0; eb < polyB.nbVertices() && !foundCommonEdge; ++eb) {
                    const auto eB = polyB.getEdge(eb);

                    // Same edge if endpoints match in same order or reversed order
                    const bool sameDir =
                        samePoint(eA.first,  eB.first)  && samePoint(eA.second, eB.second);
                    const bool oppDir  =
                        samePoint(eA.first,  eB.second) && samePoint(eA.second, eB.first);

                    if (sameDir || oppDir) {
                        commonEdge = eA;
                        foundCommonEdge = true;
                    }
                }
            }

            // If polygons share an edge => create a Link between the two servers
            if (foundCommonEdge) {
                Link *link = new Link(&ui->canvas->servers[i],
                                      &ui->canvas->servers[j],
                                      commonEdge);

                ui->canvas->links.append(link);
                ui->canvas->servers[i].links.append(link);
                ui->canvas->servers[j].links.append(link);
            }
        }
    }
}

void MainWindow::fillDistanceArray()
{
    /***********************************************************************
     * Exercise 2 — All-Pairs Shortest Paths + Routing Table
     *
     * Goal:
     *   Compute the shortest path distance between every pair of servers,
     *   and store for each source server i and target j:
     *     - total shortest distance
     *     - first Link to take (first hop) to reach j from i
     *
     * Data structures:
     *   - distanceArray[i][j] : shortest distance value (for debug/UI)
     *   - servers[i].bestDistance[j] = { firstLink, totalDistance }
     *
     * Algorithm:
     *   Floyd–Warshall (all-pairs shortest paths)
     *
     * Complexity:
     *   O(n^3), where n = number of servers
     ***********************************************************************/

    const int nServers = ui->canvas->servers.size();
    const qreal INF = 1e18;

    // Prepare distanceArray
    distanceArray.resize(nServers);
    for (int i = 0; i < nServers; ++i)
        distanceArray[i].resize(nServers);

    // Initialize bestDistance for every server
    for (auto &s : ui->canvas->servers) {
        s.bestDistance.resize(nServers);
        for (int j = 0; j < nServers; ++j)
            s.bestDistance[j] = { nullptr, 0.0 };
    }

    // dist matrix + next-hop matrix
    QVector<QVector<qreal>> dist(nServers, QVector<qreal>(nServers, INF));
    QVector<QVector<int>>   next(nServers, QVector<int>(nServers, -1));

    // Distance from a node to itself is 0
    for (int i = 0; i < nServers; ++i) {
        dist[i][i] = 0.0;
        next[i][i] = i;
    }

    // Initialize with direct edges from Links
    for (Link *l : ui->canvas->links) {
        const int a = l->getNode1()->id;
        const int b = l->getNode2()->id;
        const qreal w = l->getDistance();

        // Keep smallest edge if duplicates exist
        if (w < dist[a][b]) {
            dist[a][b] = w;
            dist[b][a] = w;
            next[a][b] = b;
            next[b][a] = a;
        }
    }

    // Floyd–Warshall: try improving dist[i][j] using intermediate k
    for (int k = 0; k < nServers; ++k) {
        for (int i = 0; i < nServers; ++i) {
            for (int j = 0; j < nServers; ++j) {
                const qreal alt = dist[i][k] + dist[k][j];
                if (alt < dist[i][j]) {
                    dist[i][j] = alt;
                    // First hop from i to j becomes the first hop from i to k
                    next[i][j] = next[i][k];
                }
            }
        }
    }

    // Build distanceArray + routing table bestDistance
    for (int i = 0; i < nServers; ++i) {
        for (int j = 0; j < nServers; ++j) {

            distanceArray[i][j] = (dist[i][j] >= INF) ? float(INF) : float(dist[i][j]);

            // Same node: no hop needed
            if (i == j) {
                ui->canvas->servers[i].bestDistance[j] = { nullptr, 0.0 };
                continue;
            }

            // Unreachable target
            if (dist[i][j] >= INF || next[i][j] == -1) {
                ui->canvas->servers[i].bestDistance[j] = { nullptr, dist[i][j] };
                continue;
            }

            // The next matrix tells the next node ID (first hop) from i toward j
            const int firstHop = next[i][j];

            // Find the actual Link* that connects i to firstHop
            Link *firstLink = nullptr;
            for (Link *l : ui->canvas->servers[i].links) {
                const int n1 = l->getNode1()->id;
                const int n2 = l->getNode2()->id;
                if ((n1 == i && n2 == firstHop) || (n2 == i && n1 == firstHop)) {
                    firstLink = l;
                    break;
                }
            }

            // Store routing decision + shortest total distance
            ui->canvas->servers[i].bestDistance[j] = { firstLink, dist[i][j] };
        }
    }

    // Debug print: complete all-pairs table
    qDebug() << "---- All-pairs shortest distances (Floyd–Warshall) ----";
    for (int i = 0; i < nServers; ++i) {
        QString line = QString("from %1: ").arg(i);
        for (int j = 0; j < nServers; ++j)
            line += QString(" %1").arg(dist[i][j], 0, 'f', 1);
        qDebug().noquote() << line;
    }
}

void MainWindow::update() {
    static int last = elapsedTimer.elapsed();
    int current = elapsedTimer.elapsed();
    int dt = current - last;
    last = current;

    for (auto &drone : ui->canvas->drones) {
        drone.overflownArea(ui->canvas->servers);
        drone.move(dt / 25.0);
    }
    ui->canvas->repaint();
}

void MainWindow::on_actionShow_graph_triggered(bool checked) {
    ui->canvas->showGraph=checked;
    ui->canvas->repaint();
}


void MainWindow::on_actionMove_drones_triggered() {
    timer = new QTimer(this);
    timer->setInterval(100);
    connect(timer,SIGNAL(timeout()),this,SLOT(update()));
    timer->start();

    elapsedTimer.start();
}


void MainWindow::on_actionQuit_triggered() {
    QApplication::quit();
}


void MainWindow::on_actionCredits_triggered() {
    QMessageBox::information(this,"About DroneAndRooms program",
                             "My tiny project.\nCredit Benoît Piranda");
}


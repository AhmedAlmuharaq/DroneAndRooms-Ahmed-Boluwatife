#include "serveranddrone.h"
#include <QDebug>

Link::Link(Server *n1,Server *n2,const QPair<Vector2D,Vector2D> &edge):
    node1(n1),node2(n2) {
    Vector2D center=0.5*(edge.first+edge.second);
    distance = (center-Vector2D(n1->position.x(),n1->position.y())).length();
    distance += (center-Vector2D(n2->position.x(),n2->position.y())).length();
    edgeCenter=QPointF(center.x,center.y);
}

void Link::draw(QPainter &painter) {
    painter.drawLine(node1->position,edgeCenter);
    painter.drawLine(node2->position,edgeCenter);
}

/* Motions of the drone to reach the "destination" position*/
void Drone::move(qreal dt)
{
    /***********************************************************************
     * Exercise 3 — Drone Animation / Navigation Logic
     *
     * Rules :
     *   1) A drone is associated with the server of its current room.
     *   2) It must first fly to that server.
     *   3) At a server: it asks for shortest path to its target.
     *      The destination becomes the door center (edgeCenter) of the first link.
     *   4) At a door: it switches room (connectedTo changes) and then flies to the
     *      server in the next room.
     *
     * Implementation strategy:
     *   We implement an implicit state machine by checking what the current
     *   destination represents (server position vs door center).
     *
     * Dependencies:
     *   - connectedTo is the current associated server (set by overflownArea()).
     *   - bestDistance[targetId].first provides the first Link* to follow.
     ***********************************************************************/

    // If the drone is not associated with any room/server yet, do nothing.
    if (connectedTo == nullptr) {
        speed = Vector2D(0, 0);
        return;
    }

    auto serverPos = [&](Server *s) -> Vector2D {
        return Vector2D(s->position.x(), s->position.y());
    };

    auto isNear = [&](const Vector2D &p) -> bool {
        return (p - position).length() < minDistance;
    };

    const Vector2D curServerPos = serverPos(connectedTo);

    // If destination is not initialized, first destination is the local server
    if ((destination - Vector2D(0,0)).length() < 1e-9) {
        destination = curServerPos;
    }

    // Determine if destination is the current server (within minDistance)
    const bool destinationIsServer = (destination - curServerPos).length() < minDistance;

    // ----------------------------------------------------------
    // STATE TRANSITIONS (server -> door -> next server)
    // ----------------------------------------------------------

    // A) We reached the current server
    if (destinationIsServer && isNear(curServerPos)) {

        // If this server is the target server => stop
        if (target != nullptr && connectedTo == target) {
            destination = position;
            speed = Vector2D(0, 0);
        } else if (target != nullptr &&
                   target->id >= 0 &&
                   target->id < connectedTo->bestDistance.size()) {

            // Ask routing table: first link toward target
            Link *nextLink = connectedTo->bestDistance[target->id].first;

            if (nextLink != nullptr) {
                // Next destination becomes the "door center" toward the next room
                destination = nextLink->getEdgeCenter();
            } else {
                // No known path => stop safely
                destination = position;
                speed = Vector2D(0, 0);
            }
        }
    }

    // B) We reached a door center => cross to neighbor room/server
    if (!destinationIsServer && isNear(destination)) {

        // Identify which link corresponds to this door (edgeCenter)
        Link *doorLink = nullptr;
        for (Link *l : connectedTo->links) {
            if ((l->getEdgeCenter() - destination).length() < minDistance) {
                doorLink = l;
                break;
            }
        }

        if (doorLink != nullptr) {
            // Switch to the other server of the link
            Server *a = doorLink->getNode1();
            Server *b = doorLink->getNode2();
            connectedTo = (connectedTo == a) ? b : a;

            // New destination: the server position inside the new room
            destination = serverPos(connectedTo);
        }
    }

    // ----------------------------------------------------------
    // PHYSICS / MOTION INTEGRATION (existing provided model)
    // ----------------------------------------------------------
    Vector2D dir = destination - position;
    const double d = dir.length();

    // If already at destination, stop
    if (d < 1e-9) {
        speed = Vector2D(0, 0);
        return;
    }

    // Slowdown when close to destination
    if (d < slowDownDistance) {
        speed = (d * speedLocal / slowDownDistance) * dir;
    } else {
        speed += (accelation * dt / d) * dir;

        // Clamp max speed
        if (speed.length() > speedMax) {
            speed.normalize();
            speed *= speedMax;
        }
    }

    // Update position
    position += (dt * speed);

    // Update orientation for icon rotation
    const double sp = speed.length();
    if (sp < 1e-9) return;

    Vector2D Vn = (1.0 / sp) * speed;
    if (Vn.y == 0) {
        azimut = (Vn.x > 0) ? -90.0 : 90.0;
    } else if (Vn.y > 0) {
        azimut = 180.0 - 180.0 * atan(Vn.x / Vn.y) / M_PI;
    } else {
        azimut = -180.0 * atan(Vn.x / Vn.y) / M_PI;
    }
}



Server* Drone::overflownArea(QList<Server>& list) {
    auto it=list.begin();
    while (it!=list.end() && !it->area.contains(position)) {
        it++;
    }
    connectedTo= it!=list.end()?&(*it):nullptr;
    return connectedTo;
}

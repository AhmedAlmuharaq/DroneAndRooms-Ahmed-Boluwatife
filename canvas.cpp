#include "canvas.h"
#include <QPainter>

Canvas::Canvas(QWidget *parent) : QWidget{parent} {
    setMouseTracking(true);
    windowOrigin={0,0};
    windowSize={1,1};
    windowScale={1.0,1.0};
    droneIconSize=64;
    droneImg.load("../../../media/drone.png");
}

void Canvas::paintEvent(QPaintEvent *) {
    const QRect rect(-droneIconSize/2,-droneIconSize/2,droneIconSize,droneIconSize);

    QPainter painter(this);
    QBrush whiteBrush(Qt::SolidPattern);
    QPen serverPen(Qt::black);
    serverPen.setWidth(3);
    QFont myFont("Arial",14,QFont::Black);
    QFontMetrics fm(myFont);
    painter.setFont(myFont);
    QPen penLink(Qt::DotLine);
    penLink.setColor(Qt::lightGray);
    // background of the window
    penLink.setWidth(3);
    whiteBrush.setColor(Qt::white);
    painter.fillRect(0,0,width(),height(),whiteBrush);

    painter.save(); // drawing area coordinate system
    painter.scale(windowScale.width(),windowScale.height());
    painter.translate(-windowOrigin);


    // drawing the servers
    QRect r;
    for (auto &s:servers) {
        painter.setBrush(s.color);
        s.area.draw(painter);

        painter.save();
        painter.translate(s.position);
        painter.setPen(serverPen);
        painter.setBrush(s.color);
        painter.drawEllipse(-25,-25,50,50);
        painter.setBrush(Qt::white);
        painter.drawEllipse(-15,-15,30,30);
        painter.setBrush(Qt::black);
        painter.drawEllipse(-5,-5,10,10);
        int tw=fm.horizontalAdvance(s.name)+2;
        int th=fm.height()+2;
        r.setRect(-tw/2,-25-th,tw,th);
        painter.drawText(r,s.name);

        painter.restore();
    }

    if (showGraph) {
        // drawing the links
        painter.setPen(penLink);
        for (auto &l:links ) {
            l->draw(painter);
        }
    }

    // drawing the drones
    painter.setPen(Qt::white);
    for (auto &d:drones) {
        painter.save();
        // place and orient the drone
        painter.translate(d.position.x,d.position.y);
        painter.rotate(d.azimut);
        painter.drawImage(rect,droneImg);

        int tw=fm.horizontalAdvance(d.name)+2;
        int th=fm.height()+2;
        r.setRect(-tw/2,-15,tw,th);
        painter.drawText(r,d.name);

        painter.restore();
    }
    painter.restore();

}

void Canvas::resizeEvent(QResizeEvent *event) {
    int w = event->size().width();
    int h = event->size().height();

    // Calcule la hauteur voulue selon le ratio
    int idealH = w * windowSize.height() / windowSize.width();

    // Si la hauteur réelle ne correspond pas, on corrige
    if (idealH < h) {
        resize(w, idealH);
        move(0,(h-idealH)/2);
    } else {
        int idealW = h * windowSize.width() / windowSize.height();
        if (idealW!=w) {
            resize(idealW, h);
            move((w-idealW)/2,0);
        }
    }

    QWidget::resizeEvent(event);

    windowScale={qreal(width())/windowSize.width(),
                   qreal(height())/windowSize.height()};
}


void Canvas::mousePressEvent(QMouseEvent *event) {

    repaint();
}


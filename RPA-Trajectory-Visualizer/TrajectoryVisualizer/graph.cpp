#include "graph.h"
#include <limits>
#include <QMouseEvent>
#include <QTextStream>
#include "trajectory.h"
#include "pointinfo.h"
#include <QBrush>
#include <QPen>
#include <QColor>
#include <math.h>

#define POINT_RADIUS 0.01f//2.5f
#define SELECTED_RADIUS (2*POINT_RADIUS)
#define SELECTION_THRESHOLD 2.0f
#define DEFAULT_MAX_TIME 99.0f
#define SCALE_INCREMENT 1.5
#define DRAW_TRIANGLES true
#define DRAW_LINES true

Graph::Graph(QFrame* frame) : QGraphicsView(frame)
{
    scene = new QGraphicsScene();
    this->setScene(scene);
    this->show();
    startTime = 0.0f;
    endTime = 99.0f;
    selected = false;
    selectedPt = new std::pair<int, int>();

    posLabel = new QLabel(this);
    posLabel->setFixedSize(100, 20);
    posLabel->setAutoFillBackground(true);
    QPalette palette = posLabel->palette();
    palette.setColor(posLabel->backgroundRole(), Qt::lightGray);
    palette.setColor(posLabel->foregroundRole(), Qt::darkGray);
    posLabel->setPalette(palette);
    posLabel->show();

    triangle = new QPolygonF();
    triangle->append(QPointF(0.025, 0.0));
    triangle->append(QPointF(-0.012, 0.012));
    triangle->append(QPointF(-0.012, -0.012));
    triangle->append(QPointF(0.025, 0.0));
    viewScore = false;
    selected2 = false;
    selectedPt2 = new std::pair<int, int>();
}

Graph::~Graph()
{
    delete scene;
}

void Graph::init(PointInfo* pointInfo, std::vector<Trajectory*>* trajectories)
{
    this->pointInfo = pointInfo;
    this->trajectories = trajectories;
}

void Graph::updateView()
{

    scene->clear();
    if (selected)
    {
        drawSelection(selectedPt->first, selectedPt->second);
    }
    if (selected2)
    {
        drawSelection(selectedPt2->first, selectedPt2->second);
    }

    for (std::vector<Trajectory*>::iterator it = trajectories->begin(); it != trajectories->end(); ++it)
    {
        Trajectory* traj = *it;
        int i = 0;
        int num = traj->getTimes()->size();
        Point prev = Point();
        bool setPrev = false;
        while (i < num){
            float time = traj->getTime(i);
            Point* pos = traj->getPosition(i);
            Point* vel = traj->getVelocity(i);
            if (time >= startTime && time <= endTime)
            {
                QPen pen;
                QBrush brush;
                if (traj->isPerson) {
                    pen = QPen(QColor(0,0,0));
                    brush = QBrush(QColor(0,0,0));
                }
                else {
                    pen = QPen(QColor(255,0,0));
                    brush = QBrush(QColor(255,0,0));

                    break;
                }
                pen.setCosmetic(true);
                if (DRAW_TRIANGLES)
                {
                    drawTriangle(pos, vel, pen, brush);
                }
                else
                {
                    scene->addEllipse(pos->x-POINT_RADIUS, pos->y-POINT_RADIUS, 2*POINT_RADIUS, 2*POINT_RADIUS, pen, brush);
                }

                if (setPrev && DRAW_LINES)
                {
                    scene->addLine(prev.x, prev.y, pos->x, pos->y, pen);
                }
                prev.x = pos->x;
                prev.y = pos->y;
                setPrev = true;
            }
            i = i + 1;
        }
        setPrev = false;
    }

}

void Graph::drawSelection(int t, int p){
    QBrush selBrush = QBrush(QColor(0,255,0));
    QPen selPen = QPen(QColor(0,255,0));
    selPen.setCosmetic(true);
    Point* selPos = trajectories->at(t)->getPosition(p);
    Point* selVel = trajectories->at(t)->getVelocity(p);
    if (DRAW_TRIANGLES)
    {
        drawTriangle(selPos, selVel, selPen, selBrush, 1.9f);
    }
    else
    {
        scene->addEllipse(selPos->x-SELECTED_RADIUS, selPos->y-SELECTED_RADIUS, 2*SELECTED_RADIUS, 2*SELECTED_RADIUS, selPen, selBrush);
    }
}

void Graph::drawTriangle(Point* pos, Point* vel, QPen pen, QBrush brush, float scale)
{
    double theta = atan(vel->y/vel->x);
    if (vel->x < 0 && vel->y < 0)
        theta = theta + M_PI;
    if (vel->x < 0 && vel->y > 0)
        theta = theta - M_PI;

    float tx = pos->x;
    float ty = pos->y;

    QPolygonF tri;
    for(QVector<QPointF>::iterator it = triangle->begin(); it != triangle->end(); ++it) {
        QPointF triPt = *it;
        double xx = cos(theta) * scale * triPt.x() + -1.0 * sin(theta) * scale * triPt.y() + tx;
        double yy = sin(theta) * scale * triPt.x() + cos(theta) * scale * triPt.y() + ty;
        tri.append(QPointF(xx,yy));
        //QTextStream(stdout) << "pt " << QString::number(pt.x()) << " " << QString::number(pt.y()) << " " << QString::number(xx) << " " << QString::number(yy) << endl;
    }
    scene->addPolygon(tri, pen, brush);
    tri.clear();
}

void Graph::mouseReleaseEvent(QMouseEvent* event)
{
    QPointF clicked = this->mapToScene(event->x(), event->y());
    float minDist = std::numeric_limits<int>::max();
    int trajIndex = -1;
    int ptIndex = -1;

    for (std::vector<Trajectory*>::iterator it = trajectories->begin(); it != trajectories->end(); ++it)
    {
        Trajectory* traj = *it;
        int i = 0;
        int num = traj->getTimes()->size();
        while (i < num){
            float time = traj->getTime(i);
            Point* pos = traj->getPosition(i);
            float dist = (pos->x -clicked.x()) * (pos->x -clicked.x()) + (pos->y -clicked.y()) * (pos->y -clicked.y());

            if (dist < minDist && dist < SELECTION_THRESHOLD && time >= startTime && time <= endTime)
            {
                minDist = dist;
                trajIndex = it - trajectories->begin();
                ptIndex = i;
            }
            i = i + 1;
        }
    }

    if (viewScore){
        if (trajIndex != -1 && ptIndex != -1)
        {
            selectPoint2(trajIndex, ptIndex);
        }
        else
        {
            deselectPoint2();
        }
    }
    else {
        if (trajIndex != -1 && ptIndex != -1)
        {
            selectPoint(trajIndex, ptIndex);
        }
        else
        {
            deselectPoint();
        }
    }
    updateView();
}

void Graph::deselectPoint()
{
    selected = false;
    selectedPt->first = -1;
    selectedPt->second = -1;
    pointInfo->clear();
    updateView();
}

void Graph::selectPoint(int t, int p)
{
    QTextStream(stdout) << "selected point " << QString::number(t) << " " << QString::number(p) << endl;
    selected = true;
    selectedPt->first = t;
    selectedPt->second = p;
    Point* pos = trajectories->at(t)->getPosition(p);
    Point* vel = trajectories->at(t)->getVelocity(p);
    float time = trajectories->at(t)->getTime(p);
    pointInfo->update(t, pos, vel, time);
    updateView();
}

void Graph::selectPoint2(int t, int p)
{
    selected2 = true;
    selectedPt2->first = t;
    selectedPt2->second = p;
    updateView();
}

void Graph::deselectPoint2() {
    selected2 = false;
    selectedPt2->first = -1;
    selectedPt2->second = -1;
    updateView();
}

void Graph::mouseMoveEvent(QMouseEvent *event)
{
    QPointF clicked = this->mapToScene(event->x(), event->y());
    posLabel->setText("(" + QString::number(clicked.x()) + "," + QString::number(clicked.y()) + ")");
}

float Graph::getMaxTime() {
    float maxTime = -1.0f;
    for (std::vector<Trajectory*>::iterator it = trajectories->begin(); it != trajectories->end(); ++it)
    {
        Trajectory* traj = *it;
        for (std::vector<float>::iterator itt = traj->getTimes()->begin(); itt != traj->getTimes()->end(); ++itt)
        {
            float t = *itt;
            if (t > maxTime)
            {
                maxTime = t;
            }
        }
    }
    if (maxTime == -1.0f)
    {
        return DEFAULT_MAX_TIME;
    }
    else
    {
        return maxTime;
    }
}

void Graph::setStartTime(float t)
{
    startTime = t;
    updateView();
}

void Graph::setEndTime(float t)
{
    endTime = t;
    updateView();
}

void Graph::wheelEvent(QWheelEvent *event)
{
    double delta = event->angleDelta().ry();
    if (delta > 0)
    {
        this->scale(SCALE_INCREMENT, SCALE_INCREMENT);
    }
    else
    {
        this->scale(1.0/SCALE_INCREMENT, 1.0/SCALE_INCREMENT);
    }
}


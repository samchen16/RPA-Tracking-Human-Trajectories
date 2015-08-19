#ifndef GRAPH_H
#define GRAPH_H

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QLabel>
#include <vector>
#include <QPolygonF>
#include "trajectory.h"
class PointInfo;


class Graph : public QGraphicsView
{
private:
    QGraphicsScene* scene;
    PointInfo* pointInfo;
    QLabel* posLabel;
    std::vector<Trajectory*>* trajectories;
    std::pair<int, int>* selectedPt;
    bool selected;
    float startTime;
    float endTime;
    QPolygonF* triangle;


public:
    Graph(QFrame* frame);
    ~Graph();
    void init(PointInfo* pointInfo, std::vector<Trajectory*>* trajectories);
    void updateView();


    // Getters and setters
    std::pair<int, int>* getSelectedPoint() { return selectedPt; }
    float getMaxTime ();
    float getStartTime () { return startTime; }
    float getEndTime () { return endTime; }
    void setStartTime (float t);
    void setEndTime (float t);
    void selectPoint(int t, int p);
    void deselectPoint();
    void drawTriangle(Point* pos, Point* vel, QPen pen, QBrush brush, float scale = 1.0f);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
};

#endif // GRAPH_H

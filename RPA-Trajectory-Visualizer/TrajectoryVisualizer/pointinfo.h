#ifndef POINTINFO_H
#define POINTINFO_H

#include <QLabel>
#include <QLineEdit>
#include "ui_trajectoryvisualizer.h"
#include "trajectory.h"

class PointInfo
{

private:
    QLabel* trajectoryLabel;
    QLabel* positionLabel;
    QLabel* velocityLabel;
    QLabel* colorLabel;
    QLabel* timeLabel;

    QLineEdit* trajectoryEdit;
    QLineEdit* positionEditX;
    QLineEdit* positionEditY;
    QLineEdit* velocityEditX;
    QLineEdit* velocityEditY;
    QLineEdit* colorEditH;
    QLineEdit* colorEditS;
    QLineEdit* colorEditV;
    QLineEdit* timeEdit;

public:
    PointInfo(Ui::TrajectoryVisualizer* ui);
    ~PointInfo();
    void update(int traj, Point* pos, Point* vel, Point* col, float time);
    void clear();
};

#endif // POINTINFO_H

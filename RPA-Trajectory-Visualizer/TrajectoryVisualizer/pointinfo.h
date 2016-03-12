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
    QLineEdit* timeEdit;

    QLineEdit* scoreEdit;
    QLineEdit* posScoreEdit;
    QLineEdit* velScoreEdit;

    float positionScore(Point* p, Point* q);
    float velocityScore(Point* p, Point* q);

public:
    PointInfo(Ui::TrajectoryVisualizer* ui);

    ~PointInfo();
    void update(int traj, Point* pos, Point* vel, float time);
    void clear();
    void updateScore(std::vector<Trajectory*>* trajectories, std::pair<int, int>* selPt, std::pair<int, int>* selPt2);
    void clearScore();
};

#endif // POINTINFO_H

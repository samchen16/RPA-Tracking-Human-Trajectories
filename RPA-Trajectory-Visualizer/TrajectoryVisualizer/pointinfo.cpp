#include "pointinfo.h"
#include "trajectory.h"
#include <math.h>

PointInfo::PointInfo(Ui::TrajectoryVisualizer* ui)
{
    trajectoryLabel = ui->trajectoryLabel;
    positionLabel = ui->positionLabel;
    velocityLabel = ui->velocityLabel;
    timeLabel = ui->timeLabel;

    trajectoryEdit = ui->trajectoryEdit;
    positionEditX = ui->positionEditX;
    positionEditY = ui->positionEditY;
    velocityEditX = ui->velocityEditX;
    velocityEditY = ui->velocityEditY;
    timeEdit = ui->timeEdit;

    scoreEdit = ui->scoreEdit;
    posScoreEdit = ui->posScoreEdit;
    velScoreEdit = ui->velScoreEdit;
}

void PointInfo::update(int traj, Point* pos, Point* vel, float time)
{
    trajectoryEdit->setText(QString::number(traj, 'f', 3));
    positionEditX->setText(QString::number(pos->x, 'f', 3));
    positionEditY->setText(QString::number(pos->y, 'f', 3));
    velocityEditX->setText(QString::number(vel->x, 'f', 3));
    velocityEditY->setText(QString::number(vel->y, 'f', 3));
    timeEdit->setText(QString::number(time));
}

void PointInfo::clear()
{
    trajectoryEdit->setText("");
    positionEditX->setText("");
    positionEditY->setText("");
    velocityEditX->setText("");
    velocityEditY->setText("");
    timeEdit->setText("");
}

void PointInfo::updateScore(std::vector<Trajectory*>* trajectories, std::pair<int, int>* selPt, std::pair<int, int>* selPt2)
{
    Point* pos1 = trajectories->at(selPt->first)->getPosition(selPt->second);
    Point* vel1 = trajectories->at(selPt->first)->getVelocity(selPt->second);
    float t1 = trajectories->at(selPt->first)->getTime(selPt->second);

    // find index of the trajectory that happened right before t1
    Trajectory* traj2 = trajectories->at(selPt2->first);
    int index2 = -1;
    int size = traj2->getTimes()->size();
    for(int i = 0; i < size; i++){
        float t2 = traj2->getTime(i);
        if (t1 > t2) {
            index2++;
        }
    }
    Point* pos2 = traj2->getPosition(index2);
    float posScore = positionScore(pos1, pos2);
    float velScore = 1.0f;
    if (index2 > 1) {
        Point* vel2 = traj2->getVelocity(index2-1);
        velScore = velocityScore(vel1, vel2);
    }
    float score = posScore + velScore;

    scoreEdit->setText(QString::number(score, 'f', 3));
    posScoreEdit->setText(QString::number(posScore, 'f', 3));
    velScoreEdit->setText(QString::number(velScore, 'f', 3));
}

void PointInfo::clearScore()
{
    scoreEdit->setText("");
    posScoreEdit->setText("");
    velScoreEdit->setText("");
}

float PointInfo::positionScore(Point* p, Point* q) {
    float x = p->x - q->x;
    float y = p->y - q->y;
    float z = p->z - q->z;
    float dist = x*x + y*y + z*z;
    return dist;

}

float PointInfo::velocityScore(Point* vel1, Point* vel2) {
    double vel1DotVel2 = vel1->x*vel2->x + vel1->y*vel2->y + vel1->z*vel2->z;
    double vel1Mag = sqrt(vel1->x*vel1->x + vel1->y*vel1->y + vel1->z*vel1->z);
    double vel2Mag = sqrt(vel2->x*vel2->x + vel2->y*vel2->y + vel2->z*vel2->z);
    float velScore = fabs( ((float) (acos(vel1DotVel2 / (vel1Mag * vel2Mag)))));
    return velScore;
}

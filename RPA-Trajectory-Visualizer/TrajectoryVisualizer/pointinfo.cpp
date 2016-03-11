#include "pointinfo.h"
#include "trajectory.h"

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

float PointInfo::positionScore(Point* p, Point* q) {
    float x = p->x - q->x;
    float y = p->y - q->y;
    float z = p->z - q->z;
    float dist = x*x + y*y + z*z;
    return dist;

}
float PointInfo::velocityScore(Point* p, Point* q) {
    return 1.0f;
}

/*void PointInfo::calculateScore() {
    Point* pos1 = trajectories->at(selectedPt->first)->getPosition(selectedPt->second);
    Point* vel1 = trajectories->at(selectedPt->first)->getVelocity(selectedPt->second);
    Point* pos2 = trajectories->at(selectedPt2->first)->getPosition(selectedPt2->second);
    Point* vel2 = trajectories->at(selectedPt2->first)->getVelocity(selectedPt2->second);
    float posScore = positionScore(pos1, pos2);
    float velScore = velocityScore(vel1, vel2);
    float score = posScore + velScore;

    scoreEdit->setText(QString::number(score, 'f', 3));
    posScoreEdit->setText(QString::number(posScore, 'f', 3));
    velScoreEdit->setText(QString::number(velScore, 'f', 3));
}*/


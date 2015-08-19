#include "pointinfo.h"
#include "trajectory.h"

PointInfo::PointInfo(Ui::TrajectoryVisualizer* ui)
{
    trajectoryLabel = ui->trajectoryLabel;
    positionLabel = ui->positionLabel;
    velocityLabel = ui->velocityLabel;
    colorLabel = ui->colorLabel;
    timeLabel = ui->timeLabel;

    trajectoryEdit = ui->trajectoryEdit;
    positionEditX = ui->positionEditX;
    positionEditY = ui->positionEditY;
    velocityEditX = ui->velocityEditX;
    velocityEditY = ui->velocityEditY;
    colorEditH = ui->colorEditH;
    colorEditS = ui->colorEditS;
    colorEditV = ui->colorEditV;
    timeEdit = ui->timeEdit;
}

void PointInfo::update(int traj, Point* pos, Point* vel, Point* col, float time)
{
    trajectoryEdit->setText(QString::number(traj, 'f', 3));
    positionEditX->setText(QString::number(pos->x, 'f', 3));
    positionEditY->setText(QString::number(pos->y, 'f', 3));
    velocityEditX->setText(QString::number(vel->x, 'f', 3));
    velocityEditY->setText(QString::number(vel->y, 'f', 3));
    colorEditH->setText(QString::number(col->x, 'f', 3));
    colorEditS->setText(QString::number(col->y, 'f', 3));
    colorEditV->setText(QString::number(col->z, 'f', 3));
    timeEdit->setText(QString::number(time));
}

void PointInfo::clear()
{
    trajectoryEdit->setText("");
    positionEditX->setText("");
    positionEditY->setText("");
    velocityEditX->setText("");
    velocityEditY->setText("");
    colorEditH->setText("");
    colorEditS->setText("");
    colorEditV->setText("");
    timeEdit->setText("");
}


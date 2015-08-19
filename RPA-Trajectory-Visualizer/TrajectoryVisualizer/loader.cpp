#include "loader.h"
#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <QTextStream>

void Loader::loadTrajectories(QString path, std::vector<Trajectory*>* trajectories)
{
    QFile inputFile(":/input.txt");
    //QFile inputFile(path);
    inputFile.open(QIODevice::ReadOnly);

    QTextStream in(&inputFile);
    Trajectory* traj;
    while (!in.atEnd())
    {
        QString line = in.readLine(); //read one line at a time
        QStringList lstLine = line.split(" ");

        QString info = lstLine.at(0);

        if (info == "person") {
            traj = new Trajectory();
            //TODO:: give id number

        }
        else if (info == "human") {
            traj->isPerson = true;
        }
        else if (info == "obstacle") {
            traj->isPerson = false;
        }
        else if (info == "end") {
            trajectories->push_back(traj);
        }
        else if (info == "t") {
            float t = lstLine.at(1).toFloat();
            traj->addTime(t);
        }
        else if (info == "p") {
            float x = lstLine.at(1).toFloat();
            float y = lstLine.at(2).toFloat();
            float z = lstLine.at(3).toFloat();
            traj->addPosition(x,y,z);
        }
        else if (info == "v") {
            float x = lstLine.at(1).toFloat();
            float y = lstLine.at(2).toFloat();
            float z = lstLine.at(3).toFloat();
            traj->addVelocity(x,y,z);
        }
        else if (info == "c") {
            float x = lstLine.at(1).toFloat();
            float y = lstLine.at(2).toFloat();
            float z = lstLine.at(3).toFloat();
            traj->addColor(x,y,z);
        }

    }
    inputFile.close();

}

void Loader::saveTrajectories(QString path, std::vector<Trajectory*>* trajectories)
{
    QFile file(path);
    file.remove();
    if (!file.open(QIODevice::ReadWrite) )
    {
        return;
    }

    QTextStream stream( &file );
    for (std::vector<Trajectory*>::iterator it = trajectories->begin(); it != trajectories->end(); ++it)
    {
        Trajectory* traj = *it;
        stream << "person" << endl;
        for (std::vector<Point*>::iterator posIt = traj->getPositions()->begin(); posIt != traj->getPositions()->end(); ++posIt)
        {
            Point* pos = *posIt;
            stream << "p " << pos->x << " " << pos->y << " " << pos->z << endl;
        }
        for (std::vector<Point*>::iterator velIt = traj->getVelocities()->begin(); velIt != traj->getVelocities()->end(); ++velIt)
        {
            Point* vel = *velIt;
            stream << "v " << vel->x << " " << vel->y << " " << vel->z << endl;
        }
        for (std::vector<Point*>::iterator colIt = traj->getColors()->begin(); colIt != traj->getColors()->end(); ++colIt)
        {
            Point* col = *colIt;
            stream << "c " << col->x << " " << col->y << " " << col->z << endl;
        }
        for (std::vector<float>::iterator timeIt = traj->getTimes()->begin(); timeIt != traj->getTimes()->end(); ++timeIt)
        {
            float time = *timeIt;
            stream << "t " << time << endl;
        }
        stream << "end" << endl;
    }

}


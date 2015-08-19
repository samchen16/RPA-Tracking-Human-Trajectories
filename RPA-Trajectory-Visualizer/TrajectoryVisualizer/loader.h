#ifndef LOADER_H
#define LOADER_H

#include <QString>
#include <vector>
#include "trajectory.h"

class Loader
{
private:

public:
    static void loadTrajectories(QString path, std::vector<Trajectory*>* trajectories);
    static void saveTrajectories(QString path, std::vector<Trajectory*>* trajectories);
};

#endif // LOADER_H

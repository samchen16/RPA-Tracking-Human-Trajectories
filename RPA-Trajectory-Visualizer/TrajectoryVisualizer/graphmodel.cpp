#include "graphmodel.h"
#include "trajectory.h"

GraphModel::GraphModel()
{

}

float GraphModel::getMaxTime() {
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
    return maxTime;
}

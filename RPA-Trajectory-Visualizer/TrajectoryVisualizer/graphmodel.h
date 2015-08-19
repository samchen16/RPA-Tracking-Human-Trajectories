#ifndef GRAPHMODEL_H
#define GRAPHMODEL_H

#include <vector>
class Trajectory;

class GraphModel
{
private:
    std::vector<Trajectory*>* trajectories;
    std::pair<int, int>* selectedPt;
    bool selected;
    float startTime;
    float endTime;

public:
    GraphModel();
    ~GraphModel();

    float getStartTime() { return startTime; }
    float getEndTime() { return endTime; }
    std::vector<Trajectory*>* getTrajectories() { return trajectories; }
    std::pair<int, int>* getSelectedPoint() { return selectedPt; }
    float getMaxTime();
};

#endif // GRAPHMODEL_H

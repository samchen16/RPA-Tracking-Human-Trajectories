#ifndef __trajectory__
#define __trajectory__

#include <vector>
#include "point.h"
#include "cluster_data.h"

/**
 * Model class representing an trajectory.
 *
 */

class Trajectory {
private:

    int id;
    std::vector<Point*>* positions;
    std::vector<Point*>* velocities;
    std::vector<Point*>* colors;
    std::vector<float>* times;    
    ClusterData* most_recent;
    
    void addPosition(float x, float y, float z);
    void addVelocity(float x, float y, float z);
    void addTime(float t) { times->push_back(t); }    

    bool is_person;
    double person_confidence;

    int frames_inactive;
    bool active;

public:
    /** The ID to assign to the next person */
    static int nextID;
    

    Trajectory();
    ~Trajectory();

    ClusterData* getClusterData() { return most_recent; }
    void setClusterData(ClusterData* cd) { most_recent = cd; }

    int getID() { return id; }

    Point* getPosition() { return positions->at(positions->size()-1); }
    Point* getVelocity() { return velocities->at(velocities->size()-1); }
    float getTime() { return times->at(times->size()-1); }

    Point* getVelocity(int t) {	return velocities->at(t); }
    Point* getPosition(int t) {	return positions->at(t); }
    float getTime(int t) { return times->at(t); }

    std::vector<Point*>* getPositions() { return positions; }
    std::vector<Point*>* getVelocities() { return velocities; }
    std::vector<float>* getTimes() { return times; }

    int getSize() { return positions->size(); } 
    void addPoint(float x, float y, float z, float t);

    void setIsPerson(bool p) { is_person = p; }
    bool isPerson() { return is_person; } 

    void setFramesInactive(int v) { frames_inactive = v; } 
    int getFramesInactive() { return frames_inactive; }

    void setActive(bool v) { active = v; }
    bool isActive() { return active; } 

    Point getApproxVelocity();

};


#endif /* defined(__trajectory__) */

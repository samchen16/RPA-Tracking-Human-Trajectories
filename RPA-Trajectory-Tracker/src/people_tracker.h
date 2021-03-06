//
//  people_tracker.h
//  PeopleTracker
//

#ifndef __people_tracker__
#define __people_tracker__

#include "constants.h"
#include "trajectory.h"
#include "cluster_data.h"
#include <vector>
#include "util.h"


class PeopleTracker {

private:
    float person_confidence;   
    int num_frames; 

    std::vector<Trajectory*>* trajectories; 
    std::vector<Trajectory*>* finished;          
    
    std::vector<ClusterData*>* clusters;
    
    float current_time;
    float minVelScore;
    float maxVelScore;
    float minColScore;
    float maxColScore;
    float minPosScore;
    float maxPosScore;

    void matchCluster(ClusterData* cd1, std::vector<Trajectory*>* trajs, bool check_person);
    void matchTrajectories();
    float velocityScore(Trajectory* traj, ClusterData* cd);
    float colorScore(Trajectory* traj, ClusterData* cd);
    float positionScore(Trajectory* traj, ClusterData* cd);
    

public:
    
    PeopleTracker();
    ~PeopleTracker();

    void setTime(float t) { current_time = t; }
    void setClusterData(std::vector<ClusterData*>* cd) { clusters = cd; }
    std::vector<Trajectory*>* getTrajectories() { return trajectories; }
    std::vector<Trajectory*>* getFinished() { return finished; }
    void setPersonConfidence(float v) { person_confidence = v; }
    void setNumFrames(int v) { num_frames = v; }

    void track();
};


#endif /* defined(__people_tracker__) */

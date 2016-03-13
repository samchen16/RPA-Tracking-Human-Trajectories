//
//  people_tracker.cpp
//  PeopleTracker
//

#include "people_tracker.h"
#include <math.h>
#include <limits>
#include <set>
#include <algorithm>

PeopleTracker::PeopleTracker() 
{
    trajectories = new std::vector<Trajectory*>();
    finished = new std::vector<Trajectory*>();
    current_time = 0.0; 
    person_confidence = -1.8f;
    num_frames = 6;
}

PeopleTracker::~PeopleTracker() 
{
   
}


void PeopleTracker::track() 
{

    //for (int i = 0; i < clusters->size(); i++) {
        //if (clusters->at(i)->getConfidence() >= person_confidence) {
            //std::cout << clusters->at(i)->getConfidence() << std::endl;
            //display_histogram_2D(*(clusters->at(i)->getColor()));
        //}
    //}

    // Set all trajectories to inactive
    for (int k = 0; k < trajectories->size(); k++)
    {   
        Trajectory* t = trajectories->at(k); 
        t->setActive(false);
    }

    // first case - nothing being tracked yet
    if (trajectories->size() == 0) {
        // just create a new trajectory for each cluster
        for (int i = 0; i < clusters->size(); i++) {
            ClusterData* cd = clusters->at(i);
            
            Trajectory* traj = new Trajectory();
            Point pos = cd->getPosition();
            traj->addPoint(pos.x, pos.y, pos.z, current_time);
            traj->setClusterData(cd);
            // can only use person confidence to determine whether the cluster is a person or obstacle
            traj->setIsPerson(cd->getConfidence() > person_confidence);
            trajectories->push_back(traj);  
        }
        return;
    }

    // second case
    matchTrajectories();

    // Get rid of trajectories that were inactive for too long
    for(std::vector<Trajectory*>::iterator it = trajectories->begin(); it != trajectories->end();) { 
        Trajectory* t = *it;			
        if (t->isActive()) {
            t->setFramesInactive(0);
        }
        else {
            t->setFramesInactive(t->getFramesInactive() + 1);
        } 

        if (t->getFramesInactive() > num_frames) {
            finished->push_back(t);
            it = trajectories->erase(it);
        }
        else {
            ++it;
        }
    }

}

float PeopleTracker::positionScore(Trajectory* traj, ClusterData* cd2) {
    ClusterData* cd1 = traj->getClusterData();
    float posScore = cluster_distance(cd1, cd2);
    if (posScore < minPosScore) {
        minPosScore = posScore;
    }
    if (posScore > maxPosScore) {
        maxPosScore = posScore;
    }
    return posScore;
}

float PeopleTracker::velocityScore(Trajectory* traj, ClusterData* cd) {
    int lookback = 0;  
    int numVel = traj->getVelocities()->size();    
    int approxIndex = numVel - lookback - 1;
        
    if (numVel == 0 ) {
        // max of acos is 1
        return 1.0f; 
    } 
    else if (approxIndex < 0) {
        approxIndex = 0;
    }
    std::cout << approxIndex << std::endl;
    Point pos1 = *(traj->getPosition());          
    Point vel1 = *(traj->getVelocity(approxIndex));

    Point pos2 = cd->getPosition();
    Point vel2 = Point();
    vel2.x = pos2.x - pos1.x;
    vel2.y = pos2.y - pos1.y;
    vel2.z = pos2.z - pos1.z; 
        
    //calculate angle between velocity vectors
    double vel1DotVel2 = vel1.x*vel2.x + vel1.y*vel2.y + vel1.z*vel2.z;
    double vel1Mag = sqrt(vel1.x*vel1.x + vel1.y*vel1.y + vel1.z*vel1.z);
    double vel2Mag = sqrt(vel2.x*vel2.x + vel2.y*vel2.y + vel2.z*vel2.z);
    float velScore = fabs(((float) (acos(vel1DotVel2 / (vel1Mag * vel2Mag)))));

    if (velScore < minVelScore) {
        minVelScore = velScore;
    }
    if (velScore > maxVelScore) {
        maxVelScore = velScore;
    }

    return velScore;
}

float PeopleTracker::colorScore(Trajectory* traj, ClusterData* cd2) {
    ClusterData* cd1 = traj->getClusterData();
    float colScore = (float) (compare_histograms(*(cd1->getColor()), *(cd2->getColor())));
    
    if (colScore < minColScore) {
        minColScore = colScore;
    }
    if (colScore > maxColScore) {
        maxColScore = colScore;
    }    

    return colScore; 
}



void PeopleTracker::matchTrajectories()
{
  // reset min and max scores
  minPosScore = std::numeric_limits<float>::min();
  maxPosScore = std::numeric_limits<float>::max();
  minVelScore = std::numeric_limits<float>::min();
  maxVelScore = std::numeric_limits<float>::max();
  minColScore = std::numeric_limits<float>::min();
  maxColScore = std::numeric_limits<float>::max();
  
  // calculate scores
  float scores[trajectories->size()][clusters->size()][3]; 
  for (int i = 0; i < trajectories->size(); i++) {
    Trajectory* traj = trajectories->at(i);
    for (int j = 0; j < clusters->size(); j++) {
        ClusterData* cd2 = clusters->at(j);
        scores[i][j][0] = positionScore(traj,cd2);
        scores[i][j][1] = velocityScore(traj,cd2);
        scores[i][j][2] = colorScore(traj,cd2);
    }
  }

  // normalize scores
  float normScores[trajectories->size()][clusters->size()]; 
  for (int i = 0; i < trajectories->size(); i++) {
    for (int j = 0; j < clusters->size(); j++) {
        float normPosScore = (scores[i][j][0] - minPosScore) / (maxPosScore - minPosScore);            
        float normVelScore = (scores[i][j][1] - minVelScore) / (maxVelScore - minVelScore);
        float normColScore = 0.0f; //(scores[i][j][2] - minColScore) / (maxColScore - minColScore);
        normScores[i][j] =  normVelScore + normColScore + normPosScore;
     }
  }

  std::set<int> matchedTrajs;
  std::set<int> matchedClusters;
  int maxIter = std::min(trajectories->size(), clusters->size());
  float distThresh = 0.25f;

  for (int it = 0; it < maxIter; it++) {  

    // find trajectory and cluster pair with best score    
    float bestScore = std::numeric_limits<float>::max();
    float bestTrajIndex = -1;
    float bestClusterIndex = -1;      
    for (int i = 0; i < trajectories->size(); i++) {
      // the trajectory has already been matched
      if (matchedTrajs.count(i) > 0) {
        continue;
      }
      for (int j = 0; j < clusters->size(); j++) {
        // the cluster has already been matched
        if (matchedClusters.count(j) > 0) {
          continue;
        }
        float score = normScores[i][j];
        if (score < bestScore && scores[i][j][0] < distThresh) {
          bestScore = score;
          bestTrajIndex = i;
          bestClusterIndex = j;
        }
      }
    }

    if (bestTrajIndex != -1 && bestClusterIndex != -1) {
      // added matched-up trajectory and cluster to matched sets
      matchedTrajs.insert(bestTrajIndex);
      matchedClusters.insert(bestClusterIndex);

      // update trajectory with cluster data
      ClusterData* bestCluster = clusters->at(bestClusterIndex);
      Trajectory* bestTraj = trajectories->at(bestTrajIndex);
      Point pos = bestCluster->getPosition();
      bestTraj->setClusterData(bestCluster);
      bestTraj->addPoint(pos.x, pos.y, pos.z, current_time);
      bestTraj->setActive(true);
      //clusters->erase(clusters->begin() + bestClusterIndex);                
    }
    else {
      // exit for-loop since there are no more pairs meeting score threshold
      break;
    }
  }


  // Create a new trajectory for each leftover cluster
  for (int j = 0; j < clusters->size(); j++) {
      if (matchedClusters.count(j) > 0) {
          continue;
      }
      ClusterData* cd = clusters->at(j);        
      Trajectory* new_traj = new Trajectory();                
      Point pos = cd->getPosition();
      new_traj->addPoint(pos.x, pos.y, pos.z, current_time);
      new_traj->setIsPerson(cd->getConfidence() > person_confidence);
      new_traj->setClusterData(cd);
      trajectories->push_back(new_traj);
  }
                 
} 

/*void PeopleTracker::matchTrajectories()
{
    for (int i = 0; i < trajectories->size(); i++) {
        Trajectory* traj = trajectories->at(i);
        ClusterData* cd1 = traj->getClusterData();

        minVelScore = std::numeric_limits<float>::min();
        maxVelScore = std::numeric_limits<float>::max();
        minColScore = std::numeric_limits<float>::min();
        maxColScore = std::numeric_limits<float>::max();

        for (int j = 0; j < clusters->size(); j++) {
            ClusterData* cd2 = clusters->at(j);
            if ((cd2->getConfidence() >= person_confidence && traj->isPerson()) || (cd2->getConfidence() < person_confidence && !traj->isPerson())) {
                
                if (cluster_distance(cd1, cd2) < 0.25)
                {
                    float velScore = velocityScore(traj,cd2);
                    float colScore = colorScore(traj,cd2);
                    float posScore = positionScore(traj,cd2);
                    cd2->setVelocityScore(velScore);
                    cd2->setColorScore(colScore);
                    cd2->setPositionScore(posScore);
                    
                }
            }
        }

        float best_score = 0.0;
        int best_index = -1;

        for (int j = 0; j < clusters->size(); j++) {
            ClusterData* cd2 = clusters->at(j);
            float normVelScore = (cd2->getVelocityScore() - minVelScore) / (maxVelScore - minVelScore);
            float normColScore = 0.0f; //(cd2->getColorScore() - minColScore) / (maxColScore - minColScore);
            float normPosScore = (cd2->getPositionScore() - minPosScore) / (maxPosScore - minPosScore);
            float score =  normVelScore + normColScore + normPosScore;
            if (score > best_score) {
                best_score = score;
                best_index = j;
            }
        }
        
        // no good match was found
        if (best_index != -1)
        {
            ClusterData* best_cluster = clusters->at(best_index);
            Point pos = best_cluster->getPosition();
            traj->setClusterData(best_cluster);
            traj->addPoint(pos.x, pos.y, pos.z, current_time);
            traj->setActive(true);
            clusters->erase(clusters->begin() + best_index);                
        }
 
    }
    
    // Create a new trajectory for each leftover cluster
    for (int i = 0; i < clusters->size(); i++) {
        ClusterData* cd = clusters->at(i);        
        Trajectory* new_traj = new Trajectory();                
        Point pos = cd->getPosition();
        new_traj->addPoint(pos.x, pos.y, pos.z, current_time);
        new_traj->setIsPerson(cd->getConfidence() > person_confidence);
        new_traj->setClusterData(cd);
        trajectories->push_back(new_traj);
    }
        
} 
*/

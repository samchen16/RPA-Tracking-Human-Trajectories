//
//  Trajectory.cpp
//  PeopleDetector
//
//  Created by Samantha Chen on 3/3/15.
//

#include "trajectory.h"
int Trajectory::nextID = 0;

/**
 * Creates a new trajectory
 */
Trajectory::Trajectory() {
    positions = new std::vector<Point*>();
    velocities = new std::vector<Point*>();
    times = new std::vector<float>();
    id = Trajectory::nextID++;
    frames_inactive = 0;
    active = true;
}

/**
 * Disposes the trajectory, releasing all resources.
 */
Trajectory::~Trajectory() {
    delete[] positions;
    delete[] velocities;
    delete[] times;
    
}

void Trajectory::addPosition(float x, float y, float z) {	
	Point* p = new Point(); 
	p->x = x; 
	p->y = y; 
	p->z = z; 
	positions->push_back(p);

}
  
void Trajectory::addVelocity(float x, float y, float z) {	
	Point* p = new Point(); 
	p->x = x; 
	p->y = y; 
	p->z = z; 
	velocities->push_back(p);    
}

void Trajectory::addPoint(float x, float y, float z, float t) {
    addPosition(x, y, z);
    addTime(t);
    
    if (positions->size() > 1)
    {
        // add velocity
        int num_positions = positions->size();
        int num_times = times->size();
        Point* p = positions->at(num_positions-1);
        Point* q = positions->at(num_positions-2);
        float delta_time = times->at(num_times-1) - times->at(num_times-2);
        float v_x = (p->x - q->x) / delta_time;
        float v_y = (p->y - q->y) / delta_time;
        float v_z = (p->z - q->z) / delta_time;        
        addVelocity(v_x, v_y, v_z);
    }
}


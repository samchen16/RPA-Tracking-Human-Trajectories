
#include "trajectory.h"

/**
 * Creates a new trajectory
 */
Trajectory::Trajectory() {
    positions = new std::vector<Point*>();
    velocities = new std::vector<Point*>();
    colors = new std::vector<Point*>();
    times = new std::vector<float>();
}


/**
 * Disposes the trajectory, releasing all resources.
 */
Trajectory::~Trajectory() {
    delete[] positions;
    delete[] velocities;
    delete[] colors;
    delete[] times;
}

/**
* Adds a position to the trajectory.
*
* @param value The position.
*/
void Trajectory::addPosition(float x, float y, float z) {
    Point* p = new Point();
    p->x = x;
    p->y = y;
    p->z = z;
    positions->push_back(p);
}

/**
* Adds a velocity to the trajectory.
*
* @return The velocity.
*/
void Trajectory::addVelocity(float x, float y, float z) {
    Point* p = new Point();
    p->x = x;
    p->y = y;
    p->z = z;
    velocities->push_back(p);
}

void Trajectory::addColor(float h, float s, float v) {
    Point* p = new Point();
    p->x = h;
    p->y = s;
    p->z = v;
    colors->push_back(p);
}

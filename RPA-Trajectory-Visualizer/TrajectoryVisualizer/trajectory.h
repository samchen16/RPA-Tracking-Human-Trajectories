#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>

struct Point {
  float x;
  float y;
  float z;
};

/**
 * Model class representing an trajectory.
 *
 */
class Trajectory
{
private:
    std::vector<Point*>* positions;
    std::vector<Point*>* velocities;
    std::vector<Point*>* colors;
    std::vector<float>* times;

public:
    bool isPerson;
    /**
     * Creates a new empty trajectory
     */
    Trajectory();

    /**
     * Disposes the trajectory, releasing all resources.
     */
    ~Trajectory();

    Point* getVelocity() { return velocities->at(velocities->size()-1); }
    Point* getPosition() { return positions->at(positions->size()-1); }
    Point* getColor() { return colors->at(colors->size()-1); }
    float getTime() { return times->at(times->size()-1); }

    Point* getVelocity(int t) { return velocities->at(t); }
    Point* getPosition(int t) { return positions->at(t); }
    Point* getColor(int t) { return colors->at(t); }
    float getTime(int t) { return times->at(t); }

    std::vector<Point*>* getPositions() { return positions; }
    std::vector<Point*>* getVelocities() { return velocities; }
    std::vector<Point*>* getColors() { return colors; }
    std::vector<float>* getTimes() { return times; }

    void addPosition(float x, float y, float z);
    void addVelocity(float x, float y, float z);
    void addColor(float h, float s, float b);
    void addTime(float t) { times->push_back(t); }

    void addPosition(Point* p) { positions->push_back(p); }
    void addVelocity(Point* p) { velocities->push_back(p); }
    void addColor(Point* p)  { colors->push_back(p); }

};

#endif // TRAJECTORY_H

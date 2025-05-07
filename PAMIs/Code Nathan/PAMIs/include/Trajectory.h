#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>
#include <LinkedList.h>

struct Waypoint {
    double x, y, theta;
};

class Trajectory {
private:
    LinkedList<Waypoint> waypoints;
    int currentIndex;

public:
    Trajectory() : currentIndex(0) {}

    void addWaypoint(double x, double y, double theta) {
        waypoints.add(Waypoint{x, y, theta});
    }

    Waypoint getNextWaypoint() {
        if (currentIndex < waypoints.size()) {
            return waypoints.get(currentIndex);
        }
        return {0, 0, 0};  // Default when finished
    }

    void advanceWaypoint() {
        if (currentIndex < waypoints.size()) {
            currentIndex++;
        }
    }

    bool isFinished() {
        return currentIndex >= waypoints.size();
    }

    int clearWaypoints() {
        int count = waypoints.size();
        waypoints.clear();
        currentIndex = 0;
        return waypoints.size() == 0 ? 1 : 0;
    }
};

#endif // TRAJECTORY_H

#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

enum class RobotState {
    IDLE,
    NAVIGATING,
    AVOIDING,
    EXECUTING_TASK,
    STOPPED
};

#endif // ROBOTSTATE_H

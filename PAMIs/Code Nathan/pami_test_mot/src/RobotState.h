#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

enum class RobotState {
    IDLE,
    NAVIGATING,
    AVOIDING,
    EXECUTING_TASK,
    STOPPED,
    DANCING
};

#endif // ROBOTSTATE_H

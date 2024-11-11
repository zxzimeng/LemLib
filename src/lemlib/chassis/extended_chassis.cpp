#include "main.h"
#include "lemlib/chassis/extended_chassis.hpp"

namespace lemlib {
void ExtendedChassis::moveToPoseWithEarlyExit(Pose pose, float timeout, MoveToPoseParams params, float exit_distance,
                                              bool async , bool degrees ) {
    if (exit_distance < 0) {
        throw std::out_of_range("Exit distance must be non-negative");
    }
    if (async) {
        pros::Task task([&]() { moveToPoseWithEarlyExit(pose, timeout, params, exit_distance, degrees, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    float expected_distance = aproximateDistanceToPoseWithBoomerang(getPose(true),
                                                                            {
                                                                                pose.x, pose.y,
                                                                                degrees
                                                                                    ? degToRad(pose.theta)
                                                                                    : pose.theta
                                                                            }, {.lead = params.lead},
                                                                            false) - exit_distance;
    moveToPose(pose.x, pose.y, pose.theta, timeout, params, false);
    waitUntil(expected_distance);
    cancelMotion();
    return;
}

void ExtendedChassis::moveToPointWithEarlyExit(Pose pose, float timeout, MoveToPointParams params, float exit_distance,
                                               bool async ) {
    if (exit_distance < 0) {
        throw std::out_of_range("Exit distance must be non-negative");
    }
    if (async) {
        pros::Task task([&]() { moveToPointWithEarlyExit(pose, timeout, params, exit_distance, async); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    float expected_distance = getPose(false).distance(pose);
    moveToPoint(pose, timeout, params, false);
    waitUntil(expected_distance);
    cancelMotion();
    return;
}

void ExtendedChassis::processMovement(movement movement_s,
                                      transform_across_field transformation ) {
    movement transformed_movement = transformMovement(calculateOffset(movement_s), transformation);
    if (std::holds_alternative<MoveToPoseParams>(transformed_movement.moveParams)) {
        MoveToPoseParams params = std::get<MoveToPoseParams>(movement_s.moveParams);
        moveToPoseWithEarlyExit(movement_s.pose, movement_s.timeout, params, movement_s.exitDistance, movement_s.async, movement_s.degrees);
    } else if (std::holds_alternative<MoveToPointParams>(transformed_movement.moveParams)) {
        MoveToPointParams params = std::get<MoveToPointParams>(movement_s.moveParams);
        moveToPointWithEarlyExit(movement_s.pose, movement_s.timeout, params, movement_s.exitDistance, movement_s.async);
    }
}
}
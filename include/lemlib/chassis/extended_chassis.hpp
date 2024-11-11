// extended_chassis.hpp
#pragma once

#include "chassis.hpp"
#include <variant>
#include "lemlib/util.hpp"
#include "lemlib/movement_utils.h"

namespace lemlib {

class ExtendedChassis : public Chassis {
public:
    // Constructor
    using Chassis::Chassis;

    // Methods for moving with early exit
    void moveToPoseWithEarlyExit(Pose pose, float timeout, MoveToPoseParams params, 
                                 float exit_distance, bool degrees = true, bool async = false);
    void moveToPointWithEarlyExit(Pose pose, float timeout, MoveToPointParams params, 
                                  float exit_distance, bool async = false);

    // Method for processing movement (with or without transformation)
    void processMovement(movement movement_s, transform_across_field transformation = {false, false});;

private:
};

}
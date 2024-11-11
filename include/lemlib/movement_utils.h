#pragma once
#include "lemlib/pose.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/chassis.hpp"
#include <variant>

namespace lemlib {

// Structure to represent movement with pose and parameters
struct movement {
    Pose pose;  // Current pose of the robot
    float offset_distance;  // Distance to offset in the direction of travel
    float perp_offset_distance;  // Distance to offset perpendicularly to the direction of travel
    std::variant<MoveToPoseParams, MoveToPointParams> moveParams;  // Parameters for movement
    float exitDistance;  // Distance to stop at
    float timeout = 4000;  // Timeout duration in milliseconds
    bool degrees = true;  // Whether angles are in degrees (true) or radians (false)
    bool async = false;  // Whether the movement should be asynchronous
};

// Structure for transforming movements across the field (e.g., mirroring)
struct transform_across_field {
    bool mirrorHorizontal;  // Whether to mirror across the horizontal axis
    bool mirrorVertical;  // Whether to mirror across the vertical axis

    // Equality operator for comparing transformations
    bool operator==(const transform_across_field &transform_across_field) const;
};

inline bool transform_across_field::operator==(const lemlib::transform_across_field &transform_across_field) const {
    return mirrorHorizontal == transform_across_field.mirrorHorizontal && mirrorVertical == transform_across_field.
           mirrorVertical;
}

float aproximateDistanceToPoseWithBoomerang(Pose current_pose, Pose pose, MoveToPoseParams params,
                                                    bool degrees=false);

double calculateArcLength(double x_start, double y_start, double x_end,
                                     double y_end, double theta_end, double d_lead, int n);

inline Pose transformPose(const Pose &pose, transform_across_field transformation) {
    Pose newPose = pose;
    if (transformation.mirrorHorizontal) {
        newPose.y *= -1;
        newPose.theta = sanitizeAngle(180 - newPose.theta, false);
    }
    if (transformation.mirrorVertical) {
        newPose.x *= -1;
        newPose.theta = sanitizeAngle(newPose.theta * -1, false);
    }
    return newPose;
}

inline movement transformMovement(movement movement_s, transform_across_field transformation) {
    movement newMovement = movement_s;
    newMovement.pose = transformPose(movement_s.pose, transformation);
    return newMovement;
}

Pose calculatePoseWithOffsetInPerpDirection(Pose pose, float offset, bool degrees);

Pose calculatePoseWithOffsetInDirection(Pose pose, float offset, bool degrees);

movement calculateOffset(movement &movement_s);
}
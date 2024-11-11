#include "main.h"
#include "lemlib/movement_utils.h"

namespace lemlib{
double calculateArcLength(double x_start, double y_start, double x_end,
                                     double y_end, double theta_end, double d_lead, int n) {
    double totalLength = 0.0;
    double h = std::sqrt(std::pow(x_start - x_end, 2) + std::pow(y_start - y_end, 2));
    double x1 = x_end - h * std::sin(theta_end) * d_lead;
    double y1 = y_end - h * std::cos(theta_end) * d_lead;

    double dt = 1.0 / n;

    for (int i = 0; i < n; i++) {
        double t1 = i * dt;
        double t2 = (i + 1) * dt;

        // Parametric X and Y values for the points at t1 and t2
        double x1_val = (1 - t1) * ((1 - t1) * x_start + t1 * x1) + t1 * ((1 - t1) * x1 + t1 * x_end);
        double y1_val = (1 - t1) * ((1 - t1) * y_start + t1 * y1) + t1 * ((1 - t1) * y1 + t1 * y_end);
        double x2_val = (1 - t2) * ((1 - t2) * x_start + t2 * x1) + t2 * ((1 - t2) * x1 + t2 * x_end);
        double y2_val = (1 - t2) * ((1 - t2) * y_start + t2 * y1) + t2 * ((1 - t2) * y1 + t2 * y_end);

        // Segment length
        double segmentLength = std::sqrt(std::pow(x2_val - x1_val, 2) + std::pow(y2_val - y1_val, 2));
        totalLength += segmentLength;
    }

    return totalLength;
}

float aproximateDistanceToPoseWithBoomerang(Pose current_pose, Pose pose, MoveToPoseParams params,
                                                    bool degrees) {
    return calculateArcLength(current_pose.x, current_pose.y, pose.x, pose.y,
                                    degrees ? degToRad(pose.theta) : pose.theta, params.lead, 8000);
}

Pose calculatePoseWithOffsetInPerpDirection(Pose pose, float offset, bool degrees) {
    // Convert theta to radians if necessary
    float angle = degrees ? degToRad(pose.theta) : pose.theta;

    // Calculate the target pose based on offset in the perpendicular direction
    Pose target_pose = {0, 0, 0};
    target_pose.x = pose.x + offset * cos(angle);
    target_pose.y = pose.y - offset * sin(angle);
    target_pose.theta = pose.theta; // Keep theta the same
    return target_pose;
}

Pose calculatePoseWithOffsetInDirection(Pose pose, float offset, bool degrees ) {
    // Convert theta to radians if necessary
    float angle = (M_PI_2 - (degrees ? degToRad(pose.theta) : pose.theta));

    // Calculate the target pose based on offset
    Pose target_pose = {0, 0, 0};
    target_pose.x = pose.x + offset * cos(angle);
    target_pose.y = pose.y + offset * sin(angle);
    target_pose.theta = degrees ? sanitizeAngle(pose.theta, false) : sanitizeAngle(pose.theta, true);
    // Keep theta the same

    return target_pose;
}

movement calculateOffset(movement &movement_s) {
    movement newMovement = movement_s;
    newMovement.pose = calculatePoseWithOffsetInPerpDirection(
        calculatePoseWithOffsetInDirection(newMovement.pose, newMovement.offset_distance, newMovement.degrees),
        newMovement.perp_offset_distance, newMovement.degrees);
    newMovement.offset_distance = 0;
    newMovement.perp_offset_distance = 0;
    return newMovement;
};}
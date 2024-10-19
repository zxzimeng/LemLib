#include <math.h>
#include <cmath>
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "lemlib/logger/logger.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/rtos.hpp"

#include <future>
#include <stdexcept>


lemlib::OdomSensors::OdomSensors(TrackingWheel* vertical1, TrackingWheel* vertical2, TrackingWheel* horizontal1,
                                 TrackingWheel* horizontal2, pros::Imu* imu)
    : vertical1(vertical1),
      vertical2(vertical2),
      horizontal1(horizontal1),
      horizontal2(horizontal2),
      imu(imu) {}

lemlib::Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth,
                               float wheelDiameter, float rpm, float horizontalDrift)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm),
      horizontalDrift(horizontalDrift) {}

lemlib::Chassis::Chassis(Drivetrain drivetrain, ControllerSettings linearSettings, ControllerSettings angularSettings,
                         OdomSensors sensors, DriveCurve* throttleCurve, DriveCurve* steerCurve)
    : drivetrain(drivetrain),
      lateralSettings(linearSettings),
      angularSettings(angularSettings),
      sensors(sensors),
      throttleCurve(throttleCurve),
      steerCurve(steerCurve),
      lateralPID(linearSettings.kP, linearSettings.kI, linearSettings.kD, linearSettings.windupRange, true),
      angularPID(angularSettings.kP, angularSettings.kI, angularSettings.kD, angularSettings.windupRange, true),
      lateralLargeExit(lateralSettings.largeError, lateralSettings.largeErrorTimeout),
      lateralSmallExit(lateralSettings.smallError, lateralSettings.smallErrorTimeout),
      angularLargeExit(angularSettings.largeError, angularSettings.largeErrorTimeout),
      angularSmallExit(angularSettings.smallError, angularSettings.smallErrorTimeout) {}

/**
 * @brief calibrate the IMU given a sensors struct
 *
 * @param sensors reference to the sensors struct
 */
void calibrateIMU(lemlib::OdomSensors& sensors) {
    int attempt = 1;
    bool calibrated = false;
    // calibrate inertial, and if calibration fails, then repeat 5 times or until successful
    while (attempt <= 5) {
        sensors.imu->reset();
        // wait until IMU is calibrated
        do pros::delay(10);
        while (sensors.imu->get_status() != pros::ImuStatus::error && sensors.imu->is_calibrating());
        // exit if imu has been calibrated
        if (!isnanf(sensors.imu->get_heading()) && !isinf(sensors.imu->get_heading())) {
            calibrated = true;
            break;
        }
        // indicate error
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        lemlib::infoSink()->warn("IMU failed to calibrate! Attempt #{}", attempt);
        attempt++;
    }
    // check if calibration attempts were successful
    if (attempt > 5) {
        sensors.imu = nullptr;
        lemlib::infoSink()->error("IMU calibration failed, defaulting to tracking wheels / motor encoders");
    }
}

void lemlib::Chassis::calibrate(bool calibrateImu) {
    // calibrate the IMU if it exists and the user doesn't specify otherwise
    if (sensors.imu != nullptr && calibrateImu) calibrateIMU(sensors);
    // initialize odom
    if (sensors.vertical1 == nullptr)
        sensors.vertical1 = new lemlib::TrackingWheel(drivetrain.leftMotors, drivetrain.wheelDiameter,
                                                      -(drivetrain.trackWidth / 2), drivetrain.rpm);
    if (sensors.vertical2 == nullptr)
        sensors.vertical2 = new lemlib::TrackingWheel(drivetrain.rightMotors, drivetrain.wheelDiameter,
                                                      drivetrain.trackWidth / 2, drivetrain.rpm);
    sensors.vertical1->reset();
    sensors.vertical2->reset();
    if (sensors.horizontal1 != nullptr) sensors.horizontal1->reset();
    if (sensors.horizontal2 != nullptr) sensors.horizontal2->reset();
    setSensors(sensors, drivetrain);
    init();
    // rumble to controller to indicate success
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

void lemlib::Chassis::setPose(float x, float y, float theta, bool radians) {
    lemlib::setPose(lemlib::Pose(x, y, theta), radians);
}

void lemlib::Chassis::setPose(Pose pose, bool radians) { lemlib::setPose(pose, radians); }

lemlib::Pose lemlib::Chassis::getPose(bool radians, bool standardPos) {
    Pose pose = lemlib::getPose(true);
    if (standardPos) pose.theta = M_PI_2 - pose.theta;
    if (!radians) pose.theta = radToDeg(pose.theta);
    return pose;
}

void lemlib::Chassis::waitUntil(float dist) {
    // do while to give the thread time to start
    do pros::delay(10);
    while (distTraveled <= dist && distTraveled != -1);
}

void lemlib::Chassis::waitUntilDone() {
    do pros::delay(10);
    while (distTraveled != -1);
}

void lemlib::Chassis::requestMotionStart() {
    if (this->isInMotion()) this->motionQueued = true; // indicate a motion is queued
    else this->motionRunning = true; // indicate a motion is running

    // wait until this motion is at front of "queue"
    this->mutex.take(TIMEOUT_MAX);

    // this->motionRunning should be true
    // and this->motionQueued should be false
    // indicating this motion is running
}

void lemlib::Chassis::endMotion() {
    // move the "queue" forward 1
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;

    // permit queued motion to run
    this->mutex.give();
}

void lemlib::Chassis::cancelMotion() {
    this->motionRunning = false;
    pros::delay(10); // give time for motion to stop
}

void lemlib::Chassis::cancelAllMotions() {
    this->motionRunning = false;
    this->motionQueued = false;
    pros::delay(10); // give time for motion to stop
}

bool lemlib::Chassis::isInMotion() const { return this->motionRunning; }

void lemlib::Chassis::resetLocalPosition() {
    float theta = this->getPose().theta;
    lemlib::setPose(lemlib::Pose(0, 0, theta), false);
}

void lemlib::Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    drivetrain.leftMotors->set_brake_mode_all(mode);
    drivetrain.rightMotors->set_brake_mode_all(mode);
}

constexpr double calculateH(double x_start, double y_start, double x_end, double y_end) {
    return std::sqrt(std::pow(x_start - x_end, 2) + std::pow(y_start - y_end, 2));
}

// Parametric functions for x(t) and y(t)
constexpr double parametricX(double t, double x_start, double x1, double x_end) {
    return (1 - t) * ((1 - t) * x_start + t * x1) + t * ((1 - t) * x1 + t * x_end);
}

constexpr double parametricY(double t, double y_start, double y1, double y_end) {
    return (1 - t) * ((1 - t) * y_start + t * y1) + t * ((1 - t) * y1 + t * y_end);
}

// Function to calculate arc length, x1, and y1
template <int n>
constexpr double calculateArcLength(double x_start, double y_start, double x_end,
                                     double y_end, double theta_end, double d_lead) {
    double h = calculateH(x_start, y_start, x_end, y_end);
    double x1 = x_end - h * std::sin(theta_end) * d_lead;
    double y1 = y_end - h * std::cos(theta_end) * d_lead;

    double totalLength = 0.0;
    double dt = 1.0 / n;

    for (int i = 0; i < n; i++) {
        double t1 = i * dt;
        double t2 = (i + 1) * dt;

        double x1_val = parametricX(t1, x_start, x1, x_end);
        double y1_val = parametricY(t1, y_start, y1, y_end);
        double x2_val = parametricX(t2, x_start, x1, x_end);
        double y2_val = parametricY(t2, y_start, y1, y_end);

        double segmentLength = std::sqrt(std::pow(x2_val - x1_val, 2) + std::pow(y2_val - y1_val, 2));
        totalLength += segmentLength;
    }

    return totalLength;
}

float lemlib::Chassis::aproximateDistanceToPoseWithBoomerang(Pose current_pose, Pose pose, MoveToPoseParams params, bool degrees=true) {
    return calculateArcLength<8000>(current_pose.x, current_pose.y, pose.x, pose.y,degrees? degToRad(pose.theta) : pose.theta, params.lead);
}

lemlib::Pose lemlib::Chassis::calculatePoseWithOffsetInDirection(Pose pose, float offset, bool degrees=true) {
    float magnitude = (Pose) {0,0,0}.distance(pose);
    float target_magnitude = magnitude+offset;
    Pose unit_vector = {sin(degrees? degToRad(pose.theta) : pose.theta)/magnitude, sin(degrees? degToRad(pose.theta) : pose.theta)/magnitude};
    Pose target_pose = unit_vector*target_magnitude;

    return target_pose;
}

void lemlib::Chassis::moveToPoseWithEarlyExit(Pose pose, float timeout, MoveToPoseParams params, float exit_distance, bool async=false, bool degrees=true) {
    if (exit_distance<0) {
        throw std::out_of_range("Exit distance must be non-negative");
    }
    if (async) {
        pros::Task task([&]() { moveToPoseWithEarlyExit(pose, timeout, params, exit_distance, degrees, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    float expected_distance = aproximateDistanceToPoseWithBoomerang(getPose(true), {pose.x, pose.y, degrees? degToRad(pose.theta) : pose.theta}, {.lead=params.lead}, false)-exit_distance;
    moveToPose(pose, timeout, params, false);
    waitUntil(expected_distance);
    cancelMotion();
    return;
}

void lemlib::Chassis::moveToPointWithEarlyExit(Pose pose, float timeout, MoveToPointParams params, float exit_distance, bool async=false) {
    if (exit_distance<0) {
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

struct movement {
    lemlib::Pose pose;
    float offset_distance;
    std::variant<lemlib::MoveToPoseParams, lemlib::MoveToPointParams> moveParams;
    float exitDistance;
    float timeout=4000;
    bool degrees=true;
    bool async=false;
};

struct transform_accross_field {
    bool mirrorHorizontal;
    bool mirrorVertical;
};

std::vector<lemlib::Chassis::movement> lemlib::Chassis::transformMovements(const std::vector<movement>& movements, transform_across_field transformation) {
    std::vector<movement> results;
    for (auto eachMovement : movements) {
        movement newMovement=eachMovement;
        if (transformation.mirrorHorizontal) {
            newMovement.pose.y*=-1;
            newMovement.pose.theta=lemlib::sanitizeAngle(180-newMovement.pose.theta);
        }
        if (transformation.mirrorVertical) {
            newMovement.pose.x*=-1;
            newMovement.pose.theta=lemlib::sanitizeAngle(newMovement.pose.theta*-1, false);
        }
    }

    return results;
}

lemlib::Pose lemlib::Chassis::transformPose(const lemlib::Pose& pose, transform_across_field transformation) {
    lemlib::Pose newPose=pose;
    if (transformation.mirrorHorizontal) {
        newPose.y*=-1;
        newPose.theta=lemlib::sanitizeAngle(180-newPose.theta);
    }
    if (transformation.mirrorVertical) {
        newPose.x*=-1;
        newPose.theta=lemlib::sanitizeAngle(newPose.theta*-1, false);
    }
    return newPose;
}

void lemlib::Chassis::processMovements(std::vector<movement>& movements) {
    for (auto eachMovement : movements) {
        if (std::holds_alternative<lemlib::MoveToPoseParams>(eachMovement.moveParams)) {
            lemlib::MoveToPoseParams params=std::get<lemlib::MoveToPoseParams>(eachMovement.moveParams);
            moveToPoseAndPointWithOffsetAndEarlyExit(eachMovement);
        }else if (std::holds_alternative<lemlib::MoveToPointParams>(eachMovement.moveParams)) {
            lemlib::MoveToPointParams params=std::get<lemlib::MoveToPointParams>(eachMovement.moveParams);
            moveToPoseAndPointWithOffsetAndEarlyExit(eachMovement);
        }
    }
}

void lemlib::Chassis::moveToPoseAndPointWithOffsetAndEarlyExit(Pose pose, float offsetDistance, float timeout, std::variant<MoveToPointParams, MoveToPoseParams> moveParams, float exit_distance, bool degrees, bool async) {
    if (std::holds_alternative<lemlib::MoveToPoseParams>(moveParams)) {
        lemlib::MoveToPoseParams params=std::get<lemlib::MoveToPoseParams>(moveParams);
        moveToPoseWithEarlyExit(calculatePoseWithOffsetInDirection(pose,offsetDistance,degrees), timeout, params, exit_distance, degrees, async);
    }else if (std::holds_alternative<lemlib::MoveToPointParams>(moveParams)) {
        lemlib::MoveToPointParams params=std::get<lemlib::MoveToPointParams>(moveParams);
        moveToPointWithEarlyExit(calculatePoseWithOffsetInDirection(pose, offsetDistance, degrees), timeout, params, exit_distance, async);
    }
}

void lemlib::Chassis::moveToPoseAndPointWithOffsetAndEarlyExit(movement &s_movement) {
    auto& moveParams = s_movement.moveParams;

    if (std::holds_alternative<MoveToPoseParams>(moveParams)) {
        MoveToPoseParams params = std::get<MoveToPoseParams>(moveParams);
        moveToPoseAndPointWithOffsetAndEarlyExit(s_movement.pose,
                                                  s_movement.offset_distance,
                                                  s_movement.timeout,
                                                  params,
                                                  s_movement.exitDistance,
                                                  s_movement.degrees,
                                                  s_movement.async);
    } else if (std::holds_alternative<MoveToPointParams>(moveParams)) {
        MoveToPointParams params = std::get<MoveToPointParams>(moveParams);
        moveToPoseAndPointWithOffsetAndEarlyExit(s_movement.pose,
                                                  s_movement.offset_distance,
                                                  s_movement.timeout,
                                                  params,
                                                  s_movement.exitDistance,
                                                  s_movement.degrees,
                                                  s_movement.async);
    }
}
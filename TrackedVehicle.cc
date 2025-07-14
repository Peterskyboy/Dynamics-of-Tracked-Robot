/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "TrackedVehicle.hh"

#include <gz/msgs/double.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/twist.pb.h>

#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/DiffDriveOdometry.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/SpeedLimiter.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

// -----------------------------------------------------------------------------
// PID Controller
// -----------------------------------------------------------------------------
class PIDController
{
public:
    PIDController(double kp = 1.0, double ki = 0.0, double kd = 0.0)
        : kp_(kp), ki_(ki), kd_(kd)
    {
        Reset();
    }

    double Update(double error, double dt)
    {
        if (dt <= 0.0) return lastOutput_;

        // Proportional
        pTerm_ = kp_ * error;

        // Integral with anti-windup
        if (!isOutputSaturated(lastOutput_))
            integral_ += error * dt;
        integral_ = std::clamp(integral_, -integralLimit_, integralLimit_);
        iTerm_ = ki_ * integral_;

        // Derivative
        double derivative = 0.0;
        if (firstRun_)
        {
            prevError_ = error;
            prevPrevError_ = error;
            lastDt_ = dt;
            firstRun_ = false;
            secondRun_ = true;
        }
        else if (secondRun_)
        {
            prevPrevError_ = prevError_;
            prevError_ = error;
            lastDt_ = dt;
            secondRun_ = false;
        }
        else
        {
            derivative = (error - prevError_) / dt;
            prevPrevError_ = prevError_;
            prevError_ = error;
            lastDt_ = dt;
        }
        dTerm_ = kd_ * derivative;

        // Output
        double output = pTerm_ + iTerm_ + dTerm_;
        lastOutput_ = std::clamp(output, -outputLimit_, outputLimit_);
        return lastOutput_;
    }

    void Reset()
    {
        integral_ = 0.0;
        prevError_ = 0.0;
        prevPrevError_ = 0.0;
        lastDt_ = 0.0;
        lastOutput_ = 0.0;
        pTerm_ = 0.0;
        iTerm_ = 0.0;
        dTerm_ = 0.0;
        firstRun_ = true;
        secondRun_ = true;
    }

    void SetGains(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void SetIntegralLimit(double limit) { integralLimit_ = limit; }
    void SetOutputLimit(double limit) { outputLimit_ = limit; }

    // Debug accessors
    double GetKp() const { return kp_; }
    double GetKi() const { return ki_; }
    double GetKd() const { return kd_; }
    double GetError() const { return prevError_; }
    double GetIntegral() const { return integral_; }
    double GetDerivative() const
    {
        if (firstRun_ || secondRun_ || lastDt_ <= 0.0)
            return 0.0;
        return (prevError_ - prevPrevError_) / lastDt_;
    }
    double GetOutput() const { return lastOutput_; }
    double GetPTerm() const { return pTerm_; }
    double GetITerm() const { return iTerm_; }
    double GetDTerm() const { return dTerm_; }

private:
    double kp_, ki_, kd_;
    double integral_ = 0.0;
    double prevError_ = 0.0;
    double prevPrevError_ = 0.0;
    double lastDt_ = 0.0;
    double lastOutput_ = 0.0;
    double integralLimit_ = 10.0;
    double outputLimit_ = 10.0;
    bool firstRun_ = true;
    bool secondRun_ = true;

    // PID terms for debugging
    double pTerm_ = 0.0;
    double iTerm_ = 0.0;
    double dTerm_ = 0.0;

    bool isOutputSaturated(double output) const
    {
        return (output >= outputLimit_) || (output <= -outputLimit_);
    }
};

// -----------------------------------------------------------------------------
// TrackedVehiclePrivate
// -----------------------------------------------------------------------------
class TrackedVehiclePrivate
{
public:
    // Command callbacks
    void OnThrottleCmd(const msgs::Double &_msg);
    void OnBrakeCmd(const msgs::Double &_msg);
    void OnSteeringCmd(const msgs::Double &_msg);
    void OnSteeringEfficiency(const msgs::Double &_msg);
    void OnDesiredVelocityCmd(const msgs::Twist &_msg);

    // Update methods
    void UpdateOdometry(const UpdateInfo &_info, const EntityComponentManager &_ecm);
    void UpdateVelocity(const UpdateInfo &_info, const EntityComponentManager &_ecm);
    void UpdatePIDControl(const UpdateInfo &_info, const EntityComponentManager &_ecm);
    void ConvertPIDOutputToCommands(double linearOutput, double angularOutput);
    void UpdateActualVelocities(const EntityComponentManager &_ecm);

    // Members
    transport::Node node;
    Entity bodyLink{kNullEntity};
    std::vector<Entity> leftTracks;
    std::vector<Entity> rightTracks;
    std::string bodyLinkName;
    std::vector<std::string> leftTrackNames;
    std::vector<std::string> rightTrackNames;
    std::unordered_map<std::string, transport::Node::Publisher> velPublishers;
    std::unordered_map<std::string, transport::Node::Publisher> corPublishers;

    double leftSpeed{0.0};
    double rightSpeed{0.0};
    double prevLeftSpeed{0.0};
    double prevRightSpeed{0.0};
    double prevLinVel{0.0};
    double prevAngVel{0.0};
    double throttleCmd{0.0};    // -1 to 1
    double brakeCmd{0.0};       // 0 to 1
    double steeringCmd{0.0};    // -1 to 1
    double desiredRotationRadiusSigned{0.0};
    math::Angle odomLeftWheelPos{0};
    math::Angle odomRightWheelPos{0};
    math::Vector3d centerOfRotation{0, 0, 0};

    // Parameters
    double tracksSeparation{1.0};
    double trackHeight{0.2};
    double trackRadius{0.1};
    double sprocketRadius{0.2};
    double steeringEfficiency{0.5};
    double slip{0.05};
    double terrainFriction{0.8};
    double mass{500.0};
    double momentOfInertia{750.0};
    double maxForce{50000.0};
    double brakeForce{10000.0};
    double trackEfficiency{0.95};
    double maxLinearVel{5.0};
    double maxAngularVel{1.5};
    double maxLinearAccel{10.0};
    double maxAngularAccel{5.0};
    double steeringDecayRate{1.0};
    double rollingResistanceCoeff{0.03};
    double maxSteeringTorque{500.0};

    Model model{kNullEntity};
    Link canonicalLink{kNullEntity};
    std::chrono::steady_clock::duration odomPubPeriod{0};
    std::chrono::steady_clock::duration lastOdomPubTime{0};
    math::DiffDriveOdometry odom;
    transport::Node::Publisher odomPub;
    transport::Node::Publisher tfPub;
    std::unique_ptr<math::SpeedLimiter> limiterLin;
    std::unique_ptr<math::SpeedLimiter> limiterAng;
    bool hasNewCommand{false};
    std::mutex mutex;
    std::string sdfFrameId;
    std::string sdfChildFrameId;
    bool debug{true};
    msgs::Marker debugMarker;
    bool disableSpeedLimiter{false};

    // PID
    PIDController linearPID;
    PIDController angularPID;
    double desiredLinearVel{0.0};
    double desiredAngularVel{0.0};
    double actualLinearVel{0.0};
    double actualAngularVel{0.0};
    bool pidEnabled{false};
    double pidUpdateRate{100.0};
    std::chrono::steady_clock::duration pidUpdatePeriod{0};
    std::chrono::steady_clock::duration lastPidUpdateTime{0};
    double maxThrottleOutput{1.0};
    double maxSteeringOutput{1.0};
    double maxBrakeOutput{1.0};
    bool pidHasNewCommand{false};

    // PID debug
    double linearError{0.0};
    double angularError{0.0};
};

// -----------------------------------------------------------------------------
// TrackedVehicle Implementation
// -----------------------------------------------------------------------------
TrackedVehicle::TrackedVehicle()
    : dataPtr(std::make_unique<TrackedVehiclePrivate>())
{
}

TrackedVehicle::~TrackedVehicle() = default;

void TrackedVehicle::Configure(const Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               EntityComponentManager &_ecm,
                               EventManager &_eventMgr)
{
    this->dataPtr->model = Model(_entity);

    if (!this->dataPtr->model.Valid(_ecm))
    {
        gzerr << "TrackedVehicle plugin should be attached to a model entity. "
              << "Failed to initialize." << std::endl;
        return;
    }

    const auto &modelName = this->dataPtr->model.Name(_ecm);

    // Canonical link
    std::vector<Entity> links = _ecm.ChildrenByComponents(_entity, components::CanonicalLink());
    if (!links.empty())
        this->dataPtr->canonicalLink = Link(links[0]);

    std::unordered_map<std::string, sdf::ElementConstPtr> tracks;

    if (_sdf->HasElement("body_link"))
        this->dataPtr->bodyLinkName = _sdf->Get<std::string>("body_link");

    // Track names
    auto sdfElem = _sdf->FindElement("left_track");
    while (sdfElem)
    {
        const auto &linkName = sdfElem->Get<std::string>("link");
        this->dataPtr->leftTrackNames.push_back(linkName);
        tracks[linkName] = sdfElem;
        sdfElem = sdfElem->GetNextElement("left_track");
    }
    sdfElem = _sdf->FindElement("right_track");
    while (sdfElem)
    {
        const auto &linkName = sdfElem->Get<std::string>("link");
        this->dataPtr->rightTrackNames.push_back(linkName);
        tracks[linkName] = sdfElem;
        sdfElem = sdfElem->GetNextElement("right_track");
    }

    // Publishers
    for (const auto &[linkName, elem] : tracks)
    {
        const auto prefix = "/model/" + modelName + "/link/" + linkName;
        auto topic = validTopic({elem->Get<std::string>("velocity_topic", prefix + "/track_cmd_vel").first});
        this->dataPtr->velPublishers[linkName] = this->dataPtr->node.Advertise<msgs::Double>(topic);

        topic = validTopic({elem->Get<std::string>("center_of_rotation_topic", prefix + "/track_cmd_center_of_rotation").first});
        this->dataPtr->corPublishers[linkName] = this->dataPtr->node.Advertise<msgs::Vector3d>(topic);
    }

    // Load parameters
    this->dataPtr->tracksSeparation = _sdf->Get<double>("tracks_separation", this->dataPtr->tracksSeparation).first;
    this->dataPtr->trackHeight = _sdf->Get<double>("track_height", this->dataPtr->trackHeight).first;
    this->dataPtr->trackRadius = _sdf->Get<double>("track_radius", this->dataPtr->trackRadius).first;
    this->dataPtr->sprocketRadius = _sdf->Get<double>("sprocket_radius", this->dataPtr->sprocketRadius).first;
    this->dataPtr->steeringEfficiency = _sdf->Get<double>("steering_efficiency", this->dataPtr->steeringEfficiency).first;
    this->dataPtr->slip = _sdf->Get<double>("slip", this->dataPtr->slip).first;
    this->dataPtr->terrainFriction = _sdf->Get<double>("terrain_friction", 0.95).first;
    this->dataPtr->mass = _sdf->Get<double>("mass", this->dataPtr->mass).first;
    this->dataPtr->momentOfInertia = _sdf->Get<double>("moment_of_inertia", this->dataPtr->momentOfInertia).first;
    this->dataPtr->maxForce = _sdf->Get<double>("max_force", 50000.0).first;
    this->dataPtr->brakeForce = _sdf->Get<double>("brake_force", this->dataPtr->brakeForce).first;
    this->dataPtr->trackEfficiency = _sdf->Get<double>("track_efficiency", 0.95).first;
    this->dataPtr->maxLinearVel = _sdf->Get<double>("max_linear_vel", this->dataPtr->maxLinearVel).first;
    this->dataPtr->maxAngularVel = _sdf->Get<double>("max_angular_vel", this->dataPtr->maxAngularVel).first;
    this->dataPtr->maxLinearAccel = _sdf->Get<double>("max_linear_accel", 10.0).first;
    this->dataPtr->maxAngularAccel = _sdf->Get<double>("max_angular_accel", 5.0).first;
    this->dataPtr->steeringDecayRate = _sdf->Get<double>("steering_decay_rate", this->dataPtr->steeringDecayRate).first;
    this->dataPtr->rollingResistanceCoeff = _sdf->Get<double>("rolling_resistance_coeff", 0.03).first;
    this->dataPtr->maxSteeringTorque = _sdf->Get<double>("max_steering_torque", this->dataPtr->maxSteeringTorque).first;
    this->dataPtr->disableSpeedLimiter = _sdf->Get<bool>("disable_speed_limiter", false).first;

    // PID Configuration
    this->dataPtr->pidEnabled = _sdf->Get<bool>("enable_pid", false).first;
    if (this->dataPtr->pidEnabled)
    {
        double linKp = _sdf->Get<double>("linear_pid_kp", 1.0).first;
        double linKi = _sdf->Get<double>("linear_pid_ki", 0.02).first;
        double linKd = _sdf->Get<double>("linear_pid_kd", 0.01).first;
        double angKp = _sdf->Get<double>("angular_pid_kp", 2.0).first;
        double angKi = _sdf->Get<double>("angular_pid_ki", 0.05).first;
        double angKd = _sdf->Get<double>("angular_pid_kd", 0.02).first;

        this->dataPtr->linearPID.SetGains(linKp, linKi, linKd);
        this->dataPtr->angularPID.SetGains(angKp, angKi, angKd);

        double linIntegralLimit = _sdf->Get<double>("linear_pid_integral_limit", 3.0).first;
        double angIntegralLimit = _sdf->Get<double>("angular_pid_integral_limit", 3.0).first;
        double linOutputLimit = _sdf->Get<double>("linear_pid_output_limit", 20.0).first;
        double angOutputLimit = _sdf->Get<double>("angular_pid_output_limit", 10.0).first;

        this->dataPtr->linearPID.SetIntegralLimit(linIntegralLimit);
        this->dataPtr->linearPID.SetOutputLimit(linOutputLimit);
        this->dataPtr->angularPID.SetIntegralLimit(angIntegralLimit);
        this->dataPtr->angularPID.SetOutputLimit(angOutputLimit);

        this->dataPtr->pidUpdateRate = _sdf->Get<double>("pid_update_rate", 100.0).first;
        std::chrono::duration<double> pidPer{1.0 / this->dataPtr->pidUpdateRate};
        this->dataPtr->pidUpdatePeriod =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(pidPer);

        this->dataPtr->maxThrottleOutput = _sdf->Get<double>("max_throttle_output", 1.0).first;
        this->dataPtr->maxSteeringOutput = _sdf->Get<double>("max_steering_output", 1.0).first;
        this->dataPtr->maxBrakeOutput = _sdf->Get<double>("max_brake_output", 1.0).first;
    }

    this->dataPtr->throttleCmd = 0.0;
    this->dataPtr->brakeCmd = 0.0;
    this->dataPtr->steeringCmd = 0.0;
    this->dataPtr->leftSpeed = 0.0;
    this->dataPtr->rightSpeed = 0.0;

    if (this->dataPtr->slip < 0.0 || this->dataPtr->slip >= 1.0)
    {
        gzerr << "Slip factor must be between 0 and less than 1, but "
              << this->dataPtr->slip << " was provided. Setting to 0.0.\n";
        this->dataPtr->slip = 0.0;
    }

    this->dataPtr->limiterLin = std::make_unique<math::SpeedLimiter>();
    this->dataPtr->limiterAng = std::make_unique<math::SpeedLimiter>();
    // ... (rest of configuration)
}

// ... (rest of the implementation)


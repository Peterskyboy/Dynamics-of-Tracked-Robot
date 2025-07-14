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
 *
 */

#ifndef GZ_SIM_SYSTEMS_TRACKEDVEHICLE_HH_
#define GZ_SIM_SYSTEMS_TRACKEDVEHICLE_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
namespace systems
{
  // Forward declaration
  class TrackedVehiclePrivate;

  /// \brief Plugin for controlling a tracked vehicle with PID control
  /// 
  /// This plugin provides dual-mode control:
  /// 1. Direct control via throttle, brake, and steering commands
  /// 2. Velocity control via cmd_vel topic (Twist messages) using PID controllers
  ///
  /// The PID controllers convert desired linear and angular velocities into 
  /// appropriate throttle and steering commands.
  ///
  /// # Parameters
  /// 
  /// - `body_link`: Name of the main body link
  /// - `left_track`: Name of the left track link(s)
  /// - `right_track`: Name of the right track link(s)
  /// - `tracks_separation`: Distance between the left and right tracks (m)
  /// - `track_height`: Height of the track (m)
  /// - `track_radius`: Radius of the track (m)
  /// - `sprocket_radius`: Radius of the sprocket (m)
  /// - `steering_efficiency`: Efficiency of steering
  /// - `slip`: Slip factor (0-1)
  /// - `terrain_friction`: Friction coefficient with terrain
  /// - `mass`: Mass of the vehicle (kg)
  /// - `moment_of_inertia`: Moment of inertia of the vehicle (kg*m^2)
  /// - `max_force`: Maximum force applied to tracks (N)
  /// - `brake_force`: Maximum brake force (N)
  /// - `track_efficiency`: Efficiency of tracks
  /// - `max_linear_vel`: Maximum linear velocity (m/s)
  /// - `max_angular_vel`: Maximum angular velocity (rad/s)
  /// - `max_linear_accel`: Maximum linear acceleration (m/s^2)
  /// - `max_angular_accel`: Maximum angular acceleration (rad/s^2)
  /// - `steering_decay_rate`: Rate at which steering decays
  /// - `rolling_resistance_coeff`: Rolling resistance coefficient
  /// - `max_steering_torque`: Maximum steering torque (Nm)
  /// - `throttle_topic`: Topic for throttle commands
  /// - `brake_topic`: Topic for brake commands
  /// - `steering_topic`: Topic for steering commands
  /// - `velocity_cmd_topic`: Topic for velocity commands (Twist)
  /// - `odom_topic`: Topic for odometry
  /// - `tf_topic`: Topic for tf
  /// - `enable_pid`: Enable PID control (true/false)
  /// - `linear_pid_kp`: Proportional gain for linear velocity PID
  /// - `linear_pid_ki`: Integral gain for linear velocity PID
  /// - `linear_pid_kd`: Derivative gain for linear velocity PID
  /// - `angular_pid_kp`: Proportional gain for angular velocity PID
  /// - `angular_pid_ki`: Integral gain for angular velocity PID
  /// - `angular_pid_kd`: Derivative gain for angular velocity PID
  /// - `linear_pid_integral_limit`: Limit for linear velocity PID integrator
  /// - `angular_pid_integral_limit`: Limit for angular velocity PID integrator
  /// - `linear_pid_output_limit`: Limit for linear velocity PID output
  /// - `angular_pid_output_limit`: Limit for angular velocity PID output
  /// - `pid_update_rate`: Update rate for PID controllers (Hz)
  /// - `debug`: Enable debug output (true/false)
  class TrackedVehicle
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate,
        public gz::sim::ISystemPostUpdate
  {
    /// \brief Constructor
    public: TrackedVehicle();

    /// \brief Destructor
    public: ~TrackedVehicle() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                EntityComponentManager &_ecm,
                EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<TrackedVehiclePrivate> dataPtr;
  };
}
}
}

#endif
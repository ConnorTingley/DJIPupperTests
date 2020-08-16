#include "DriveSystem.h"

#include <Streaming.h>

#include "Utils.h"

DriveSystem::DriveSystem() : front_bus_(), rear_bus_() {
  control_mode_ = DriveControlMode::kIdle;
  fault_current_ = 10.0;  // TODO: don't make this so high at default
  fault_position_ = 3.5;  // TODO: make this a param
  fault_velocity_ =
      100000.0;  // TODO: make this a param, set to something more reasonable.
  max_current_ = 0.0;  // TODO: make this a parameter
  position_reference_.fill(0.0);
  velocity_reference_.fill(0.0);
  current_reference_.fill(
      0.0);  // todo, log the commanded current even when in position PID mode
  active_mask_.fill(false);
  zero_position_.fill(0.0);
}

void DriveSystem::CheckForCANMessages() {
  front_bus_.pollCAN();
  rear_bus_.pollCAN();
}

DriveControlMode DriveSystem::CheckErrors() {
  for (size_t i = 0; i < kNumActuators; i++) {
    // check positions
    if (abs(GetActuatorPosition(i)) > fault_position_) {
      return DriveControlMode::kError;
    }
    // check velocities
    if (abs(GetActuatorVelocity(i)) > fault_velocity_) {
      return DriveControlMode::kError;
    }
  }
  return DriveControlMode::kIdle;
}

void DriveSystem::SetIdle() { control_mode_ = DriveControlMode::kIdle; }

void DriveSystem::ZeroCurrentPosition() {
  SetZeroPositions(GetRawActuatorPositions());
}

void DriveSystem::SetZeroPositions(ActuatorPositionVector zero) {
  zero_position_ = zero;
}

void DriveSystem::SetAllPositions(ActuatorPositionVector pos) {
  control_mode_ = DriveControlMode::kPositionControl;
  position_reference_ = pos;
}

void DriveSystem::SetPosition(uint8_t i, float position_reference) {
  control_mode_ = DriveControlMode::kPositionControl;
  position_reference_[i] = position_reference;
}

void DriveSystem::SetPositionKp(uint8_t i, float kp) {
  position_gains_[i].kp = kp;
}

void DriveSystem::SetPositionKd(uint8_t i, float kd) {
  position_gains_[i].kd = kd;
}

void DriveSystem::SetAllPositionKp(float kp) {
  for (uint8_t i = 0; i < kNumActuators; i++) {
    SetPositionKp(i, kp);
  }
}

void DriveSystem::SetAllPositionKd(float kd) {
  for (uint8_t i = 0; i < kNumActuators; i++) {
    SetPositionKd(i, kd);
  }
}

void DriveSystem::SetAllPositionGains(PDGains gains) {
  SetAllPositionKp(gains.kp);
  SetAllPositionKd(gains.kd);
}

void DriveSystem::SetCurrent(uint8_t i, float current_reference) {
  control_mode_ = DriveControlMode::kCurrentControl;
  current_reference_[i] = current_reference;
}

void DriveSystem::SetFaultCurrent(float fault_current) {
  fault_current_ = fault_current;
}

void DriveSystem::SetMaxCurrent(float max_current) {
  max_current_ = max_current;
}

// TODO: Add saturation to PID!
void DriveSystem::Update() {
  // If there are errors, put the system in the error state.
  if (CheckErrors() == DriveControlMode::kError) {
    control_mode_ = DriveControlMode::kError;
  }

  switch (control_mode_) {
    case DriveControlMode::kError: {
      // TODO: Add some sort of error handling.
      Serial << "ERROR" << endl;
      CommandIdle();
      break;
    }
    case DriveControlMode::kIdle: {
      CommandIdle();
      break;
    }
    case DriveControlMode::kPositionControl: {
      ActuatorCurrentVector pd_current;
      for (size_t i = 0; i < kNumActuators; i++) {
        PD(pd_current[i], GetActuatorPosition(i), GetActuatorVelocity(i),
           position_reference_[i], velocity_reference_[i], position_gains_[i]);
      }
      CommandCurrents(pd_current);
      break;
    }
    case DriveControlMode::kCartesianPositionControl: {
      // TODO Implement arbitrary axis compliance, but probably just xyz
      // This would need kinematic knowledge however, so maybe this code
      // should not belong here?
      ActuatorCurrentVector cartesian_pd_currents;
      for (int leg_index = 0; leg_index < 4; leg_index++) {
        RobotSide side = GetLegSide(leg_index);
        auto joint_angles = GetLegJointAngles(leg_index);
        auto jac = LegJacobian(joint_angles, leg_parameters_, side);

        auto measured_positions =
            LegKinematics(joint_angles, leg_parameters_, side);
        auto measured_velocities = jac * GetLegJointVelocities(leg_index);
        auto reference_positions = GetLegCartesianPositionReference(leg_index);
        auto reference_velocities = GetLegCartesianVelocityReference(leg_index);

        PDGains3x3 gains;
        gains.kp = DiagonalMatrix3x3(
            100);  // [A/m] probably want around 1A/cm = 100A/m
        gains.kd = DiagonalMatrix3x3(1.0);  // [A/m/s]
        auto cartesian_forces =
            PDControl3(measured_positions, measured_velocities,
                       reference_positions, reference_velocities, gains);
        auto joint_torques = ~jac * cartesian_forces;

        // Ensures that the direction of the force is preserved
        // when motors saturate
        float norm = InfinityNorm3(joint_torques);
        if (norm > max_current_) {
          joint_torques /= norm * max_current_;
        }

        cartesian_pd_currents[3 * leg_index] = joint_torques(0);
        cartesian_pd_currents[3 * leg_index + 1] = joint_torques(1);
        cartesian_pd_currents[3 * leg_index + 2] = joint_torques(2);
      }
      CommandCurrents(cartesian_pd_currents);
      break;
    }
    case DriveControlMode::kCurrentControl: {
      CommandCurrents(current_reference_);
      break;
    }
  }
}

C610Bus<CAN1> &DriveSystem::FrontBus() { return front_bus_; }

C610Bus<CAN2> &DriveSystem::RearBus() { return rear_bus_; }

// TODO: make the activation simpler
void DriveSystem::ActivateActuator(uint8_t i) { active_mask_[i] = true; }

void DriveSystem::DeactivateActuator(uint8_t i) { active_mask_[i] = false; }

void DriveSystem::SetActivations(ActuatorActivations acts) {
  active_mask_ = acts;  // Is this a copy?
}

void DriveSystem::ActivateAll() {
  for (size_t i = 0; i < kNumActuators; i++) {
    ActivateActuator(i);
  }
}

void DriveSystem::DeactivateAll() {
  for (size_t i = 0; i < kNumActuators; i++) {
    DeactivateActuator(i);
  }
}

void DriveSystem::CommandIdle() {
  ActuatorCurrentVector currents;
  currents.fill(0.0);
  CommandCurrents(currents);
}

void DriveSystem::CommandCurrents(ActuatorCurrentVector currents) {
  ActuatorCurrentVector current_command =
      Constrain(currents, -max_current_, max_current_);
  // TODO: kind of redundant with constrain right above
  if (Maximum(current_command) > fault_current_ ||
      Minimum(current_command) < -fault_current_) {
    Serial << "Requested current too large. Erroring out." << endl;
    control_mode_ = DriveControlMode::kError;
    return;
  }

  current_command = MaskArray(current_command, active_mask_);
  std::array<int32_t, kNumActuators> currents_mA;
  currents_mA = ConvertToFixedPoint(current_command, kMilliAmpPerAmp);

  front_bus_.commandTorques(currents_mA[0], currents_mA[1], currents_mA[2],
                            currents_mA[3], 0);
  front_bus_.commandTorques(currents_mA[4], currents_mA[5], 0, 0, 1);
  rear_bus_.commandTorques(currents_mA[6], currents_mA[7], currents_mA[8],
                           currents_mA[9], 0);
  rear_bus_.commandTorques(currents_mA[10], currents_mA[11], 0, 0, 1);
}

C610 DriveSystem::GetController(uint8_t i) {
  // TODO put these constants somewhere else
  if (i >= 0 && i <= 5) {
    return front_bus_.get(i);
  } else if (i >= 6 && i <= 11) {
    return rear_bus_.get(i - 6);
  } else {
    Serial << "Invalid actuator index. Must be 0<=i<=11." << endl;
    // Should not be used
    // Maybe should error out here
    return C610();
  }
}

float DriveSystem::GetRawActuatorPosition(uint8_t i) {
  return (GetController(i).counts() / kCountsPerRad);
}

ActuatorPositionVector DriveSystem::GetRawActuatorPositions() {
  ActuatorPositionVector pos;
  for (size_t i = 0; i < pos.size(); i++) {
    pos[i] = GetRawActuatorPosition(i);
  }
  return pos;
}

float DriveSystem::GetActuatorPosition(uint8_t i) {
  return GetRawActuatorPosition(i) - zero_position_[i];
}

ActuatorPositionVector DriveSystem::GetActuatorPositions() {
  ActuatorPositionVector pos;
  for (size_t i = 0; i < pos.size(); i++) {
    pos[i] = GetActuatorPosition(i);
  }
  return pos;
}

float DriveSystem::GetActuatorVelocity(uint8_t i) {
  return GetController(i).rpm() / kRPMPerRadS;
}

float DriveSystem::GetActuatorCurrent(uint8_t i) {
  return GetController(i).torque() / kMilliAmpPerAmp;
}

BLA::Matrix<3> DriveSystem::GetLegJointAngles(uint8_t i) {
  return {GetActuatorPosition(3 * i), GetActuatorPosition(3 * i + 1),
          GetActuatorPosition(3 * i + 2)};
}

BLA::Matrix<3> DriveSystem::GetLegJointVelocities(uint8_t i) {
  return {GetActuatorVelocity(3 * i), GetActuatorVelocity(3 * i + 1),
          GetActuatorVelocity(3 * i + 2)};
}

// Get the cartesian reference position for leg i.
BLA::Matrix<3> DriveSystem::GetLegCartesianPositionReference(uint8_t i) {
  return {cartesian_position_reference_(3 * i),
          cartesian_position_reference_(3 * i + 1),
          cartesian_position_reference_(3 * i + 2)};
}

// Return the cartesian reference velocity for leg i.
BLA::Matrix<3> DriveSystem::GetLegCartesianVelocityReference(uint8_t i) {
  return {cartesian_velocity_reference_(3 * i),
          cartesian_velocity_reference_(3 * i + 1),
          cartesian_velocity_reference_(3 * i + 2)};
}

RobotSide DriveSystem::GetLegSide(uint8_t leg_index) {
  if (leg_index == 0 || leg_index == 2) {
    return RobotSide::kRight;
  } else if (leg_index == 1 || leg_index == 3) {
    return RobotSide::kLeft;
  }
  return RobotSide::kLeft;  // TODO error here
}

void DriveSystem::PrintHeader(DrivePrintOptions options) {
  char delimiter = '\t';
  if (options.time) {
    Serial << "T" << delimiter;
  }
  for (size_t i = 0; i < kNumActuators; i++) {
    if (!active_mask_[i]) continue;
    if (options.positions) {
      Serial << "p[" << i << "]" << delimiter;
    }
    if (options.velocities) {
      Serial << "v[" << i << "]" << delimiter;
    }
    if (options.currents) {
      Serial << "I[" << i << "]" << delimiter;
    }
    if (options.position_references) {
      Serial << "pr[" << i << "]" << delimiter;
    }
    if (options.velocity_references) {
      Serial << "vr[" << i << "]" << delimiter;
    }
    if (options.current_references) {
      Serial << "Ir[" << i << "]" << delimiter;
    }
  }
  Serial << endl;
}

void DriveSystem::PrintStatus(DrivePrintOptions options) {
  char delimiter = '\t';
  if (options.time) {
    Serial << millis() << delimiter;
  }
  for (uint8_t i = 0; i < kNumActuators; i++) {
    if (!active_mask_[i]) continue;
    if (options.positions) {
      Serial.print(GetActuatorPosition(i), 3);
      Serial << delimiter;
    }
    if (options.velocities) {
      Serial.print(GetActuatorVelocity(i), 1);
      Serial << delimiter;
    }
    if (options.currents) {
      Serial.print(GetActuatorCurrent(i), 3);
      Serial << delimiter;
    }
    if (options.position_references) {
      Serial.print(position_reference_[i], 3);
      Serial << delimiter;
    }
    if (options.velocity_references) {
      Serial.print(velocity_reference_[i], 1);
      Serial << delimiter;
    }
    if (options.current_references) {
      Serial.print(current_reference_[i], 3);
      Serial << delimiter;
    }
  }
  Serial << endl;
}
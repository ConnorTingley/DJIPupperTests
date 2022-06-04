#include "PID.h"

#include <BasicLinearAlgebra.h>

void PD(float &torque_command, float measurement_pos, float measurement_vel,
        float reference_pos, float reference_vel, PDGains gains) {
  torque_command = gains.kp * (reference_pos - measurement_pos) +
                   gains.kd * (reference_vel - measurement_vel);
}

Print &operator<<(Print &stream, const PDGains &gains) {
  stream << "kp: " << gains.kp << " kd: " << gains.kd;
  return stream;
}

BLA::Matrix<3> PDControl3(BLA::Matrix<3> measured_position,
                          BLA::Matrix<3> measured_velocity,
                          BLA::Matrix<3> reference_position,
                          BLA::Matrix<3> reference_velocity, PDGains3x3 gains) {
  return gains.kp * (reference_position - measured_position) +
         gains.kd * (reference_velocity - measured_velocity);
}

BLA::Matrix<3> YPRToError( float y, float p, float r){
  float ca = cos(y);
  float sa = sin(y);
  float cb = cos(p);
  float sb = sin(p);
  float cc = cos(r);
  float sc = sin(r);
  BLA::Matrix<3,3> R = {
    ca*cb, ca*sb*sc-sa*cc, ca*sb*cc+sa*sc,
    sa*cb, sa*sb*sc+ca*cc, sa*sb*cc-ca*sc,
    -sb, cb*sc, cb*cc};
  
  float TR = R(0,0) + R(1,1) + R(2,2); 
  float theta = acos((TR - 1) / 2);
  BLA::Matrix<3> w = {R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1)};
  w = w * 1/(2*sin(theta));
  return w * theta;
}

BLA::Matrix<3> AngularPDControl(
        float measured_yaw,        // [radians]
        float measured_pitch,      // [radians]
        float measured_roll,       // [radians]
        float measured_yaw_rate,   // [radians/sec]
        float measured_pitch_rate, // [radians/sec]
        float measured_roll_rate,   // [radians/sec]) 
        
        float desired_pitch,      // [radians]
        float desired_roll,       // [radians]
        float desired_yaw_rate,   // [radians/sec]
        float desired_pitch_rate, // [radians/sec]
        float desired_roll_rate,   // [radians/sec]
        PDGains3x3 gains
        ){
  float yaw_dsp_dot =  desired_yaw_rate - measured_yaw_rate;
  float pitch_dsp_dot = desired_pitch_rate - measured_pitch_rate;
  float roll_dsp_dot = desired_roll_rate - measured_roll_rate;

  BLA::Matrix<3> rot_error = YPRToError(0, desired_pitch - measured_pitch, desired_roll - measured_roll);

  BLA::Matrix<3> delta_w = {roll_dsp_dot, pitch_dsp_dot, yaw_dsp_dot};

  BLA::Matrix<3> torques = delta_w * gains.kda + rot_error * gains.kpa;

  return torques;
}
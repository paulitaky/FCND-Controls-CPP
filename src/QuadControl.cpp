#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

float QuadControl::ClipTilt(float x)
{
    if (x < -maxTiltAngle) {
        return -maxTiltAngle;
    } else if (x > maxTiltAngle) {
        return maxTiltAngle;
    } else {
        return x;
    }
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // linear equation system setup with tau_x, tau_y, tau_z, dependent on F_i with 
  // i = 1,...,4, L, kappa = k_m/k_f, and c = F_1 + ..., 
  // solved after F_i with i = 1,..,4 with linear equation solver
  float l = L / pow(2.f, 0.5);
  float c = collThrustCmd;
  float f_1 = (c * kappa * l + kappa * (momentCmd.x + momentCmd.y) - l * momentCmd.z) / (4.0 * kappa * l);
  float f_2 = (c * kappa * l - kappa * (momentCmd.x - momentCmd.y) + l * momentCmd.z) / (4.0 * kappa * l);
  float f_3 = (c * kappa * l - kappa * (momentCmd.x + momentCmd.y) - l * momentCmd.z) / (4.0 * kappa * l);
  float f_4 = (c * kappa * l + kappa * (momentCmd.x - momentCmd.y) + l * momentCmd.z) / (4.0 * kappa * l);

  // thrust 3 and 4 have to be switched due to rotor setup
  cmd.desiredThrustsN[0] = f_1; // front left
  cmd.desiredThrustsN[1] = f_2; // front right
  cmd.desiredThrustsN[2] = f_4; // rear left
  cmd.desiredThrustsN[3] = f_3; // rear right

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  V3F rate_error = pqrCmd - pqr;
  V3F mom_iner(Ixx, Iyy, Izz);

  momentCmd = mom_iner * kpPQR * rate_error;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float c_d = collThrustCmd / mass;
  float desired_R13, desired_R23, pitch_rate, roll_rate;

  if (collThrustCmd > 0.0) {
      desired_R13 = -ClipTilt(accelCmd[0] / c_d);
      desired_R23 = -ClipTilt(accelCmd[1] / c_d);

      float b_dot_x_c = kpBank * (R(0,2) - desired_R13);
      float b_dot_y_c = kpBank * (R(1,2) - desired_R23);

      pitch_rate = (1 / R(2,2)) * (-R(1,0) * b_dot_x_c + R(0,0) * b_dot_y_c);
      roll_rate = (1 / R(2,2)) * (-R(1,1) * b_dot_x_c + R(0,1) * b_dot_y_c);
  } else {
      pitch_rate = 0.0;
      roll_rate = 0.0;
  }

  pqrCmd[0] = pitch_rate;
  pqrCmd[1] = roll_rate;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float pos_diff = kpPosZ * (posZCmd - posZ);
  float vel_diff = kpVelZ * (velZCmd - velZ);
  integratedAltitudeError += (posZCmd - posZ) * dt;

  float u_bar_1 = pos_diff + vel_diff + (KiPosZ * integratedAltitudeError) + accelZCmd;
  float vertical_accel = (u_bar_1 - 9.81f) / R(2,2);

  // Limit ascent and descent rate
  if (vertical_accel > (maxDescentRate / dt)) {
      // descent mode
      vertical_accel = maxDescentRate / dt;
  } else if (vertical_accel < (-maxAscentRate / dt)) {
      // ascent mode
      vertical_accel = -maxAscentRate / dt;
  }

  thrust = -mass * vertical_accel;

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  V3F pos_diff;
  pos_diff[0] = kpPosXY * (posCmd[0] - pos[0]);
  pos_diff[1] = kpPosXY * (posCmd[1] - pos[1]);

  // limit horizontal velocity
  float velCmd_norm = pow(pow(velCmd[0], 2) + pow(velCmd[1], 2), 0.5);
  if (velCmd_norm > maxSpeedXY) {
      velCmd[0] = (velCmd[0] * maxSpeedXY) / velCmd_norm;
      velCmd[1] = (velCmd[1] * maxSpeedXY) / velCmd_norm;
  }

  V3F vel_diff;
  vel_diff[0] = kpVelXY * (velCmd[0] - vel[0]);
  vel_diff[1] = kpVelXY * (velCmd[1] - vel[1]);

  accelCmd += pos_diff + vel_diff;

  // limit horizontal acceleration
  float accelCmd_norm = pow(pow(accelCmd[0], 2) + pow(accelCmd[1], 2), 0.5);
  if (accelCmd_norm > maxAccelXY) {
      accelCmd[0] = (accelCmd[0] * maxAccelXY) / accelCmd_norm;
      accelCmd[1] = (accelCmd[1] * maxAccelXY) / accelCmd_norm;
  }

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // limit commanded yaw to range from 0 to 2*pi
  float yawCmd_lim = fmodf(yawCmd, 2.0*M_PI);

  float yaw_error = yawCmd_lim - yaw;

  if (yaw_error > M_PI) {
      yaw_error = yaw_error - 2.0 * M_PI;
  } else if (yaw_error < -M_PI) {
      yaw_error = yaw_error + 2.0 * M_PI;
  }

  yawRateCmd = kpYaw * yaw_error;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}

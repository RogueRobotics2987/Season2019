/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/autoDriveForward.h"

autoDriveForward::autoDriveForward(double duration, double speed) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::driveTrain.get());
  SetTimeout(duration);
  mySpeed = speed;
}

// Called just before this Command runs the first time
void autoDriveForward::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void autoDriveForward::Execute() {
  Robot::driveTrain->differentialDrive1->ArcadeDrive(mySpeed, 0.0, 0.0);
}

// Make this return true when this Command no longer needs to run execute()
bool autoDriveForward::IsFinished() {
  return IsTimedOut(); 
}

// Called once after isFinished returns true
void autoDriveForward::End() {
  Robot::driveTrain->differentialDrive1->ArcadeDrive(0.0, 0.0, 0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void autoDriveForward::Interrupted() {
  End();
}

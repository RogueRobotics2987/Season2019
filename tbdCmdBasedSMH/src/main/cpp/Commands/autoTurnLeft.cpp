/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/autoTurnLeft.h"

autoTurnLeft::autoTurnLeft(double duration, double rotation) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::driveTrain.get());
  SetTimeout(duration);
  myRotation = rotation;
}

// Called just before this Command runs the first time
void autoTurnLeft::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void autoTurnLeft::Execute() {
  Robot::driveTrain->differentialDrive1->ArcadeDrive(0.0, myRotation, 0.0);
}

// Make this return true when this Command no longer needs to run execute()
bool autoTurnLeft::IsFinished() { 
  return IsTimedOut(); 
}

// Called once after isFinished returns true
void autoTurnLeft::End() {
  Robot::driveTrain->differentialDrive1->ArcadeDrive(0.0, 0.0, 0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void autoTurnLeft::Interrupted() {
  End();
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/lift.h"

lift::lift(double duration, double speed) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::arm.get());
  SetTimeout(duration);
  mySpeed = speed;
}

// Called just before this Command runs the first time
void lift::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void lift::Execute() {
  // Robot::arm->armMotor(-1.0);
  Robot::arm->armMotor->Set(mySpeed);
}

// Make this return true when this Command no longer needs to run execute()
bool lift::IsFinished() { 
  return IsTimedOut();
}

// Called once after isFinished returns true
void lift::End() {
   Robot::arm->armMotor->Set(0);

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void lift::Interrupted() {
   End();
}

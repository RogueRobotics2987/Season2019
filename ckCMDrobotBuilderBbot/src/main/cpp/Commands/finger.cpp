/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/finger.h"

finger::finger(double duration, double speed) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::finger.get());
  SetTimeout(duration);
  mySpeed = speed;
}

// Called just before this Command runs the first time
void finger::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void finger::Execute() {
  Robot::finger->fingerMotor->Set(mySpeed);
}

// Make this return true when this Command no longer needs to run execute()
bool finger::IsFinished() { 
  return IsTimedOut(); 
  }

// Called once after isFinished returns true
void finger::End() {
  Robot::finger->fingerMotor->Set(0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void finger::Interrupted() {
  End();
}

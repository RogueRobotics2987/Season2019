/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/take.h"

take::take(double duration, double speed) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::intake.get());
  SetTimeout(duration);
  mySpeed = speed;

}

// Called just before this Command runs the first time
void take::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void take::Execute() {
  Robot::intake->intake->Set(mySpeed);
}

// Make this return true when this Command no longer needs to run execute()
bool take::IsFinished() { 
  return IsTimedOut(); 
}

// Called once after isFinished returns true
void take::End() {
  Robot::intake->intake->Set(0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void take::Interrupted() {
  End();
}

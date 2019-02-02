/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/driveForward.h"

driveForward::driveForward() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(Robot::drivetrain.get());
}

// Called just before this Command runs the first time
void driveForward::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void driveForward::Execute() {
  Robot::drivetrain->differentialDrive1->ArcadeDrive(1.0, 0.0, 0.0);

}

// Make this return true when this Command no longer needs to run execute()
bool driveForward::IsFinished() { return false; }

// Called once after isFinished returns true
void driveForward::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void driveForward::Interrupted() {}

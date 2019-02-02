/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/jCommands.h"

jCommands::jCommands() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_subsystem);

  // l1 = new WPI_TalonSRX(1);
  // l2 = new WPI_TalonSRX(2);
  // r1 = new WPI_TalonSRX(3);
  // r2 = new WPI_TalonSRX(4);

}

// Called just before this Command runs the first time
void jCommands::Initialize() {
    // l1->Set(ControlMode::PercentOutput, 0);
    // l2->Set(ControlMode::PercentOutput, 0);
    // r1->Set(ControlMode::PercentOutput, 0);
    // r2->Set(ControlMode::PercentOutput, 0);

}

// Called repeatedly when this Command is scheduled to run
void jCommands::Execute() {
  // l1.Set(joytick0.GetY);
  m_drive.DriveCartesian(joystick0.GetY(), joystick0.GetX(), joystick0.GetZ());

}

// Make this return true when this Command no longer needs to run execute()
bool jCommands::IsFinished() { return false; }

// Called once after isFinished returns true
void jCommands::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void jCommands::Interrupted() {}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Robot.h"
#include <frc/commands/Command.h>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include <frc/WPILib.h>


using namespace frc;

class jCommands : public frc::Command {
public:
  jCommands();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
public:
  frc::Joystick joystick0{0}; 
  WPI_TalonSRX l1 = {20};
  WPI_TalonSRX l2 = {22};
  WPI_TalonSRX r1 = {27};
  WPI_TalonSRX r2 = {21};
  frc::MecanumDrive m_drive{l1, l2, r1, r2};

  // frc::MecanumDrive drive(l1, l2, r1, r2); 
};
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <frc/joystick.h>
#include <frc/wpilib.h> 
#include <iostream>
#include <frc/TimedRobot.h> 
#include <rev/SparkMax.h> 
#include <rev/CANSparkMax.h> 
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/drive/DifferentialDrive.h>
#include "ctre/Phoenix.h" 
WPI_TalonSRX Talon1 = {25}; 
rev::CANSparkMax SparkMax1(41, rev::CANSparkMax::MotorType::kBrushless);
frc::Joystick stick{0}; 
rev::CANEncoder encoder = SparkMax1.GetEncoder(); 
rev::CANPIDController PIDController = SparkMax1.GetPIDController(); 

double kP = (1/30.0); 
double kMin = (-1); 
double kMax = (1); 
void Robot::RobotInit() {
  PIDController.SetI(0);
  PIDController.SetD(0);
  PIDController.SetIZone(0);
  PIDController.SetFF(0);
  PIDController.SetP(kP); 
  PIDController.SetOutputRange(kMin, kMax); 
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutNumber("Max Output", kMax); 
  frc::SmartDashboard::PutNumber("Min Output", kMax); 
  frc::SmartDashboard::PutNumber("P Gain", kP);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
  
   double p = frc::SmartDashboard::GetNumber("P Value", 0);
   double min = frc::SmartDashboard::GetNumber("Min", 0); 
   double max = frc::SmartDashboard::GetNumber("Max", 0); 
   double rotations = frc::SmartDashboard::GetNumber("Rotations", 0); 
   // SparkMax1.Set(-stick.GetY());
    //Talon1.Set(stick.GetY()); 
    frc::SmartDashboard::PutNumber("Encoder Position", encoder.GetPosition());
    if((p != kP)) { PIDController.SetP(p); kP = p; }
    if(kMax != max || kMin != min){ 
      PIDController.SetOutputRange(min, max); 
      kMax = max; 
      kMin = min; 
    }
  PIDController.SetReference(rotations, rev::ControlType::kPosition); 
  frc::SmartDashboard::PutNumber("SetPoint", rotations);
  frc::SmartDashboard::PutNumber("ProcessVariable", encoder.GetPosition());

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

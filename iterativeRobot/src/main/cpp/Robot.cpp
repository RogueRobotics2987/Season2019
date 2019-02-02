/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <frc/drive/DifferentialDrive.h>
#include <frc/drive/DifferentialDrive.h>
#include <networktables/NetworkTable.h> 
#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"
#include <frc/Wpilib.h>
#include <frc/Timer.h>
#include "PIDControl.h"
  double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
  frc::Timer* myTimer; 
PIDControl LeftControl; 
PIDControl RightControl; 
frc::Joystick stick{0}; 
rev::CANSparkMax LeftBack(40, rev::CANSparkMax::MotorType::kBrushless); 
rev::CANEncoder LeftBackEncoder = LeftBack.GetEncoder(); 
rev::CANPIDController LeftBackPID = LeftBack.GetPIDController(); 
rev::CANSparkMax LeftFront(42, rev::CANSparkMax::MotorType::kBrushless); 
rev::CANEncoder LeftFrontEncoder = LeftFront.GetEncoder(); 
rev::CANPIDController LeftFrontPID = LeftFront.GetPIDController(); 
rev::CANSparkMax RightBack(43, rev::CANSparkMax::MotorType::kBrushless); 
rev::CANEncoder RightBackEncoder = RightBack.GetEncoder(); 
rev::CANPIDController RightBackPID = RightBack.GetPIDController(); 
rev::CANSparkMax RightFront(44, rev::CANSparkMax::MotorType::kBrushless); 
rev::CANEncoder RightFrontEncoder = RightFront.GetEncoder(); 
rev::CANPIDController RightFrontPID = RightFront.GetPIDController(); 

//frc::DifferentialDrive drive{LeftFront, RightBack}; 
void Robot::RobotInit() {
  myTimer = new frc::Timer(); 
    LeftBackPID.SetP(kP);
    LeftBackPID.SetI(kI);
    LeftBackPID.SetD(kD);
    LeftBackPID.SetIZone(kIz);
    LeftBackPID.SetFF(kFF);

    LeftFrontPID.SetP(kP);
    LeftFrontPID.SetI(kI);
    LeftFrontPID.SetD(kD);
    LeftFrontPID.SetIZone(kIz);
    LeftFrontPID.SetFF(kFF);
    LeftFrontPID.SetOutputRange(kMinOutput, kMaxOutput);

    RightFrontPID.SetP(kP);
    RightFrontPID.SetI(kI);
    RightFrontPID.SetD(kD);
    RightFrontPID.SetIZone(kIz);
    RightFrontPID.SetFF(kFF);
    RightFrontPID.SetOutputRange(kMinOutput, kMaxOutput);

    RightBackPID.SetP(kP);
    RightBackPID.SetI(kI);
    RightBackPID.SetD(kD);
    RightBackPID.SetIZone(kIz);
    RightBackPID.SetFF(kFF);
    RightBackPID.SetOutputRange(kMinOutput, kMaxOutput);



    LeftBackPID.SetOutputRange(kMinOutput, kMaxOutput);
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  frc::SmartDashboard::PutNumber("P Gain", kP);
  frc::SmartDashboard::PutNumber("I Gain", kI);
  frc::SmartDashboard::PutNumber("D Gain", kD);
  frc::SmartDashboard::PutNumber("I Zone", kIz);
  frc::SmartDashboard::PutNumber("Feed Forward", kFF);
  frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
  frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
  frc::SmartDashboard::PutNumber("Set Rotations", 0);
}
double getNewPos(double newCmd){ 
  static double lastCmd = 0; 
  if(newCmd > lastCmd + 1){ 
    lastCmd = lastCmd + 1; 
  }
  else { 
    lastCmd = newCmd; 
  }
  return lastCmd; 
}
// double getNewPosStick(double speed){ 

//   static double prevTime = myTimer->Get(); 
//   static double oldPos = 0; 

//   double curTime = myTimer->Get(); 
//   double dt = curTime - prevTime; 
//   double targetPos = oldPos + speed * dt; 
//   prevTime = curTime; 
//   oldPos = targetPos;
//   return targetPos; 
// }

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
  
   frc::SmartDashboard::PutNumber("Max Speed", 1.0); 
  myTimer->Start(); 
  LeftFront.SetInverted(true); 
  //RightFront.Follow(RightBack);
 //LeftBack.Follow(LeftFront); 
  //RightBack.SetInverted(true); 

 // std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); 
}

void Robot::TeleopPeriodic() {
  double maxSpeed = frc::SmartDashboard::GetNumber("Max Speed", 0); 
  double posLeft = LeftControl.getNewPosStick(stick.GetY()*maxSpeed*2, myTimer->Get());
  double posRight = RightControl.getNewPosStick(stick.GetY()*maxSpeed, myTimer->Get()); 
  frc::SmartDashboard::PutNumber("Motor Temp", RightBack.GetMotorTemperature());

    double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);
  //  drive.ArcadeDrive(-stick.GetY(), stick.GetZ()); 
    frc::SmartDashboard::PutNumber("SetPoint", rotations);
 frc::SmartDashboard::PutNumber("RB encoder", RightBackEncoder.GetPosition());
 //RightFrontPID.SetReference(rotations, rev::ControlType::kPosition);
 //LeftBackPID.SetReference(rotations, rev::ControlType::kPosition);
 LeftFrontPID.SetReference( posLeft, rev::ControlType::kPosition); 
 RightBackPID.SetReference(posRight, rev::ControlType::kPosition);

 frc::SmartDashboard::PutNumber("LF encoder", LeftFrontEncoder.GetPosition()); 
 frc::SmartDashboard::PutNumber("Left Reference Value", posLeft);  
 frc::SmartDashboard::PutNumber("Right Reference Value", posRight);

}

void Robot::TestPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

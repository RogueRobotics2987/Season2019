/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.   //                                                            */
/*----------------------------------------------------------------------------*/
#include <frc/drive/DifferentialDrive.h>
#include "Robot.h"
#include "AHRS.h" 
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"
#include <frc/Wpilib.h>
#include <frc/Timer.h>
#include "PIDControl.h"
#include "ctre/Phoenix.h" 
#include "xBoxControl.h" 
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include "Limelight.h"
double kP = 0.1, kI = 1e-4, kD = 0, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;
// 6 sparks 2 talons 
// Volts upward range from .25 to .375 
// Talon for intake 
// Arm positionor spark 
// winch motor spark
// levitate motors talon 
// Drives spark

LimelightControl LimelightCam; 
xBoxController auxControl; 
frc::Compressor *shooterCompressor = new frc::Compressor; 
//frc::DoubleSolenoid gearShifts{0,0}; 
frc::Solenoid shooter{0}; 
WPI_TalonSRX intake = {32}; 
PIDControl winchControlPID;
// Spark
rev::CANSparkMax winchChannel(41, rev::CANSparkMax::MotorType::kBrushless);  
AHRS* myAhrs; 
frc::Timer* myTimer; 
PIDControl LeftControl; 
PIDControl RightControl; 
frc::Joystick stick{0}; 
frc::Joystick xBox{1}; 
rev::CANSparkMax ArmMotor(45, rev::CANSparkMax::MotorType::kBrushless); 
// Not added yet rev::CANSparkMax ArmMotor(0, rev::CANSparkMax::MotorType::kBrushless);
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
//TalonSRX* talon_foot = new TalonSRX{17}; 
rev::CANEncoder winchEncoder = winchChannel.GetEncoder(); 
 rev::CANPIDController winchPID = winchChannel.GetPIDController(); 
rev::CANPIDController armPID = ArmMotor.GetPIDController(); 
PIDControl armControlPID; 
frc::DifferentialDrive drive{LeftFront, RightBack}; 
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

  winchPID.SetI(kI);
  winchPID.SetD(kD);
  winchPID.SetIZone(kIz);
  winchPID.SetFF(kFF);
  winchPID.SetOutputRange(kMinOutput, kMaxOutput);

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
  ArmMotor.SetSmartCurrentLimit(1); 
  frc::SmartDashboard::PutNumber("Winch Reference", 0); 
  
    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);

	winchChannel.SetSmartCurrentLimit(6); 
   frc::SmartDashboard::PutNumber("Max Speed", 1.0); 
   myTimer->Reset();
  myTimer->Start(); 
  LeftFront.SetInverted(true); 
  myAhrs = new AHRS(SerialPort::kMXP); 
  myAhrs -> Reset(); 

  //RightFront.Follow(RightBack);
 //LeftBack.Follow(LeftFront); 
  //RightBack.SetInverted(true); 

}

void Robot::TeleopPeriodic() {
  // read PID coefficients from SmartDashboard
  double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  double max = frc::SmartDashboard::GetNumber("Max Output", 0);
  double min = frc::SmartDashboard::GetNumber("Min Output", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if((p != kP))   { winchPID.SetP(p); kP = p; }
  if((i != kI))   { winchPID.SetI(i); kI = i; }
  if((d != kD))   { winchPID.SetD(d); kD = d; }
  if((iz != kIz)) { winchPID.SetIZone(iz); kIz = iz; }
  if((ff != kFF)) { winchPID.SetFF(ff); kFF = ff; }
  if((max != kMaxOutput) || (min != kMinOutput)) { winchPID.SetOutputRange(min, max); kMinOutput = min; kMaxOutput = max; }

  shooterCompressor->SetClosedLoopControl(true); 
  frc::SmartDashboard::PutNumber("Winch encoder position", winchEncoder.GetPosition()); 
  frc::SmartDashboard::PutNumber("Winch Applied Output ", winchChannel.GetAppliedOutput()); 

  double maxSpeed = frc::SmartDashboard::GetNumber("Max Speed", 0); 
  double posLeft = LeftControl.getNewPosStick(stick.GetY()*maxSpeed*2, myTimer->Get());
  double posRight = RightControl.getNewPosStick(stick.GetY()*maxSpeed, myTimer->Get()); 
  frc::SmartDashboard::PutNumber("Motor Temp", RightBack.GetMotorTemperature());
  double targetPitch = -3; 
  double errorPitch = myAhrs->GetPitch() - targetPitch; 
  // talon_foot.Set(.4 * errorPitch); 
   if(targetPitch*errorPitch > 0){ 
     //talon_foot.ConfigContinuousCurrentLimit(.2, 20); 
   }
  if(targetPitch*errorPitch < 0){ 
  //  talon_foot.ConfigContinuousCurrentLimit(20, 20);
  }
  //Add directional based current limit 
  // if(errorYaw * errorYaw > 0) then setlimit(2A) else setlimit(20A)
  
 //frc::SmartDashboard::PutNumber("Talon Current", talon_foot->GetOutputCurrent());

 
   
   //talon_foot->Set(ControlMode::PercentOutput, stick.GetY()); 
    double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);
  //  drive.ArcadeDrive(-stick.GetY(), stick.GetZ()); 
    frc::SmartDashboard::PutNumber("SetPoint", rotations);
 frc::SmartDashboard::PutNumber("RB encoder", RightBackEncoder.GetPosition());
 LeftFrontPID.SetReference( posLeft, rev::ControlType::kPosition); 
 RightBackPID.SetReference(posRight, rev::ControlType::kPosition);

 frc::SmartDashboard::PutNumber("LF encoder", LeftFrontEncoder.GetPosition()); 
 frc::SmartDashboard::PutNumber("Left Reference Value", posLeft);  
 frc::SmartDashboard::PutNumber("Right Reference Value", posRight);
 frc::SmartDashboard::PutNumber("Gyro Value", myAhrs->GetAngle()); 
 frc::SmartDashboard::PutNumber("Yaw Value", myAhrs->GetYaw()); 
 frc::SmartDashboard::PutNumber("Pitch Value", myAhrs->GetPitch()); 
 frc::SmartDashboard::PutNumber("Solenoid On", shooter.Get()); 
//auxControl.moveWinch(&xBox, &winchChannel, &winchPID, &winchControlPID, myTimer->Get()); 
auxControl.MoveWinchNoStick(&winchPID); 
auxControl.intakeControl(&xBox, &intake); 
//LimelightCam.visionOn(&stick); 
//LimelightCam.visionMove(&drive, &stick); 
frc::SmartDashboard::PutNumber("xBox right axis", xBox.GetRawAxis(3)); 
frc::SmartDashboard::PutNumber("xBox left axis", xBox.GetRawAxis(2)); 
//ArmMotor.Set(stick.GetY()*0.1); 
armPID.SetReference(armControlPID.getNewPosStick(stick.GetY(), myTimer->Get()), rev::ControlType::kPosition); 
// if(stick.GetRawButton(3)){ 
//   gearShifts.Set(frc::DoubleSolenoid::Value::kOff); 
// }
// else if(stick.GetRawButton(4)){ 
//   gearShifts.Set(frc::DoubleSolenoid::Value::kOff); 
// }
}

void Robot::TestPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

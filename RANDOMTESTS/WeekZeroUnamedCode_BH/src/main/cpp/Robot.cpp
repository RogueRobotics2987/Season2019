/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.   //                                                            */
/*----------------------------------------------------------------------------*/
#include <frc/drive/DifferentialDrive.h>
#include "Robot.h"
#include "AHRS.h"   //teseted
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"  //test
#include <frc/Wpilib.h>
#include <frc/Timer.h>
#include "PIDControl.h"
#include "ctre/Phoenix.h"  //test
#include "xBoxControl.h" 
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include "Limelight.h"
#include "Scalar.h" 
double kP = 0.01, kI = 1e-5, kD = 0, kIz = 1.5, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
double AkP = 4e-4, AkI = .8e-6, AkD = 0, AkIz = 5, AkFF = 0; // D could be .0056 

double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;
double fullMatch = 120; 
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
frc::DoubleSolenoid gearShifts{2,3}; 
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
TalonSRX* talon_foot = new TalonSRX{17}; 
rev::CANEncoder winchEncoder = winchChannel.GetEncoder(); 
 rev::CANPIDController winchPID = winchChannel.GetPIDController(); 
rev::CANPIDController armPID = ArmMotor.GetPIDController(); 
rev::CANEncoder armEncoder = ArmMotor.GetEncoder(); 
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

  frc::SmartDashboard::PutNumber("P Gain", kP);
  frc::SmartDashboard::PutNumber("I Gain", kI);
  frc::SmartDashboard::PutNumber("D Gain", kD);
  frc::SmartDashboard::PutNumber("I Zone", kIz);
  frc::SmartDashboard::PutNumber("Feed Forward", kFF);
  frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
  frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
  //frc::SmartDashboard::PutNumber("Set Rotations", 0);

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
double getLeftMotor(double xSpeed, double zRotation){ 
  double maxInput = std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);
  double leftMotorOutput; 
  double rightMotorOutput; 
  if (xSpeed >= 0.0) {
    // First quadrant, else second quadrant
    if (zRotation >= 0.0) {
      leftMotorOutput = maxInput;
      rightMotorOutput = xSpeed - zRotation;
    } else {
      leftMotorOutput = xSpeed + zRotation;
      rightMotorOutput = maxInput;
    }
  } else {
    // Third quadrant, else fourth quadrant
    if (zRotation >= 0.0) {
      leftMotorOutput = xSpeed + zRotation;
      rightMotorOutput = maxInput;
    } else {
      leftMotorOutput = maxInput;
      rightMotorOutput = xSpeed - zRotation;
    }
  }
return leftMotorOutput; 

}
double getRightMotor(double xSpeed, double zRotation){ 
  double maxInput = std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);
  double leftMotorOutput; 
  double rightMotorOutput; 

  if (xSpeed >= 0.0) {
    // First quadrant, else second quadrant
    if (zRotation >= 0.0) {
      leftMotorOutput = maxInput;
      rightMotorOutput = xSpeed - zRotation;
    } else {
      leftMotorOutput = xSpeed + zRotation;
      rightMotorOutput = maxInput;
    }
  } else {
    // Third quadrant, else fourth quadrant
    if (zRotation >= 0.0) {
      leftMotorOutput = xSpeed + zRotation;
      rightMotorOutput = maxInput;
    } else {
      leftMotorOutput = maxInput;
      rightMotorOutput = xSpeed - zRotation;
    }
  }

return rightMotorOutput; 
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
  TeleopInit(); 
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
  TeleopPeriodic(); 
  // Code for auto hand movement 
  // static double oldTime = myTimer->Get(); 
  // if(myTimer->Get() - oldTime > 5 && myTimer->Get() - oldTime < 7){ my_foot->Set(45);}
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  shooterCompressor->SetClosedLoopControl(true); 
  armPID.SetSmartMotionMaxVelocity(kMaxVel);
  armPID.SetSmartMotionMinOutputVelocity(kMinVel); 
  armPID.SetSmartMotionMaxAccel(kMaxAcc); 
  armPID.SetSmartMotionAllowedClosedLoopError(kAllErr); 
  LimelightCam.LimelightInit();

  ArmMotor.SetSmartCurrentLimit(20); 
  armPID.SetP(AkP);
  armPID.SetI(AkI);
  armPID.SetD(AkD);
  armPID.SetIZone(AkIz);
  armPID.SetFF(AkFF);
  armPID.SetOutputRange(kMinOutput, kMaxOutput);
    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);

    frc::SmartDashboard::PutNumber("aP Gain", AkP);
    frc::SmartDashboard::PutNumber("aI Gain", AkI);
    frc::SmartDashboard::PutNumber("aD Gain", AkD);
    frc::SmartDashboard::PutNumber("aI Zone", AkIz);
    frc::SmartDashboard::PutNumber("aFeed Forward", AkFF);
    
 // RightFront.SetOpenLoopRampRate(1); 
	winchChannel.SetSmartCurrentLimit(60); 
   frc::SmartDashboard::PutNumber("Max Speed", 1.0); 
   myTimer->Reset();
  myTimer->Start(); 
  LeftFront.SetInverted(true); 
  myAhrs = new AHRS(SerialPort::kMXP); 
  myAhrs -> Reset(); 

 RightFront.Follow(RightBack);
 LeftBack.Follow(LeftFront); 
  //RightBack.SetInverted(true); 
  armControlPID.init(armEncoder.GetPosition(), myTimer->Get());

}

void Robot::TeleopPeriodic() {
frc::SmartDashboard::PutNumber("Arm Current", ArmMotor.GetOutputCurrent()); 
  frc::SmartDashboard::PutNumber("Arm Encoder Value", armEncoder.GetPosition()); 
    frc::SmartDashboard::PutNumber("Winch Current", winchChannel.GetOutputCurrent());

  // RightBack.SetRampRate(rampRate); 
  // LeftFront.SetRampRate(rampRate); 
  // LeftFront.SetParameter(rev::CANSparkMaxLowLevel::)
  // LeftFront.SetParameter(rev::CANSparkMaxLowLevel::ConfigParameter::kClosedLoopRampRate, rampRate); 
  // RightBack.SetParameter(rev::CANSparkMaxLowLevel::ConfigParameter::kClosedLoopRampRate, rampRate); 
 //RightBack.Set(stick.GetY()); // 43 
//  RightFront.Set(stick.GetY()); // 44 
 //LeftBack.Set(stick.GetY()); // 40 
 
 //LeftFront.Set(stick.GetY()); // 42 
 // LeftFront.Set(stick.GetY()); 
  // read PID coefficients from SmartDashboard
  double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  double max = frc::SmartDashboard::GetNumber("Max Output", 0);
  double min = frc::SmartDashboard::GetNumber("Min Output", 0);

  double Ap = frc::SmartDashboard::GetNumber("aP Gain", 0);
  double Ai = frc::SmartDashboard::GetNumber("aI Gain", 0);
  double Ad = frc::SmartDashboard::GetNumber("aD Gain", 0);
  double Aiz = frc::SmartDashboard::GetNumber("aI Zone", 0);
  double Aff = frc::SmartDashboard::GetNumber("aFeed Forward", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if((p != kP))   { RightBackPID.SetP(p); kP = p; }
  if((i != kI))   { RightBackPID.SetI(i); kI = i; }
  if((d != kD))   { RightBackPID.SetD(d); kD = d; }
  if((iz != kIz)) { RightBackPID.SetIZone(iz); kIz = iz; }
  if((ff != kFF)) { RightBackPID.SetFF(ff); kFF = ff; }
  if((max != kMaxOutput) || (min != kMinOutput)) { RightBackPID.SetOutputRange(min, max); kMinOutput = min; kMaxOutput = max; }
   if((p != kP))   { LeftFrontPID.SetP(p); kP = p; }
  if((i != kI))   { LeftFrontPID.SetI(i); kI = i; }
  if((d != kD))   { LeftFrontPID.SetD(d); kD = d; }
  if((iz != kIz)) { LeftFrontPID.SetIZone(iz); kIz = iz; }
  if((ff != kFF)) { LeftFrontPID.SetFF(ff); kFF = ff; }
  if((max != kMaxOutput) || (min != kMinOutput)) { LeftFrontPID.SetOutputRange(min, max); kMinOutput = min; kMaxOutput = max; }

   if((Ap != AkP))   { armPID.SetP(Ap); AkP = Ap; }
  if((Ai != AkI))   { armPID.SetI(Ai); AkI = Ai; }
  if((Ad != AkD))   { armPID.SetD(Ad); AkD = Ad; }
  if((Aiz != AkIz)) { armPID.SetIZone(Aiz); AkIz = Aiz; }
  if((Aff != kFF)) { armPID.SetFF(Aff); AkFF = Aff; }
  if((max != kMaxOutput) || (min != kMinOutput)) { armPID.SetOutputRange(min, max); kMinOutput = min; kMaxOutput = max; }
  

  frc::SmartDashboard::PutNumber("Winch encoder position", winchEncoder.GetPosition()); 
  frc::SmartDashboard::PutNumber("Winch Applied Output ", winchChannel.GetAppliedOutput());
 // frc::SmartDashboard::PutNumber("Left Motor Speed Output", getLeftMotor(stick.GetY(), stick.GetZ()));
  //frc::SmartDashboard::PutNumber("Right Motor Speed Output", getRightMotor(stick.GetY(), stick.GetZ()));
  double maxSpeed = frc::SmartDashboard::GetNumber("Max Speed", 0); 
  double posLeft = LeftControl.getNewPosStick(stick.GetY()*maxSpeed, myTimer->Get());
  double posRight =  RightControl.getNewPosStick(stick.GetY()*maxSpeed, myTimer->Get()); 
  double posRightTurn =  RightControl.getNewPosStick(getRightMotor(stick.GetY(), stick.GetZ())* maxSpeed, myTimer->Get()); 
  double posLeftTurn = LeftControl.getNewPosStick(getLeftMotor(stick.GetY(), stick.GetZ())* maxSpeed, myTimer->Get());
  frc::SmartDashboard::PutNumber("Winch Temp", winchChannel.GetMotorTemperature());
  //Right back left front 
  //RightBackPID.SetReference(posRightTurn, rev::ControlType::kPosition); 
  // LeftFrontPID.SetReference(posLeftTurn, rev::ControlType::kPosition); 
//  double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);
  //RightBackPID.SetReference(posRight, rev::ControlType::kPosition); 
  //RightBackPID.SetReference(rotations, rev::ControlType::kPosition); 
  //LeftFrontPID.SetReference(rotations, rev::ControlType::kPosition); 
  //RightBackPID.SetReference(stick.GetY() * 30, rev::ControlType::kPosition); 
  // LeftFrontPID.SetReference(stick.GetY() * 30, rev::ControlType::kPosition); 
  //LeftFrontPID.SetReference(posLeft, rev::ControlType::kPosition); 
  // talon_foot.Set(.4 * errorPitch); 

  //Add directional based current limit 
  // if(errorYaw * errorYaw > 0) then setlimit(2A) else setlimit(20A)

  //frc::SmartDashboard::PutNumber("Talon Current", talon_foot->GetOutputCurrent());



  //talon_foot->Set(ControlMode::PercentOutput, stick.GetY()); 
  //  drive.ArcadeDrive(-stick.GetY(), stick.GetZ()); 
  double timeRemaining = fullMatch - myTimer->Get(); 
  frc::SmartDashboard::PutNumber("Time Remaining In Match", timeRemaining); 
 frc::SmartDashboard::PutNumber("RB encoder", RightBackEncoder.GetPosition());
 

 frc::SmartDashboard::PutNumber("LF encoder", LeftFrontEncoder.GetPosition()); 
 //frc::SmartDashboard::PutNumber("Left Reference Value", posLeft);  
// frc::SmartDashboard::PutNumber("Right Reference Value", posRight);
 frc::SmartDashboard::PutNumber("Pitch Value", myAhrs->GetPitch()); 
 frc::SmartDashboard::PutNumber("Solenoid On", shooter.Get()); 
//auxControl.moveWinch(&xBox, &winchChannel, &winchPID, &winchControlPID, myTimer->Get()); 
//auxControl.MoveWinchNoStick(&winchPID); 
//auxControl.intakeControl(&xBox, &intake); 
auxControl.setShooter(&xBox, &shooter, &stick, myTimer); 
//auxControl.moveArm(&xBox, &armPID, &armControlPID, armEncoder, myTimer->Get()); 
//ArmMotor.Set(xBox.GetRawAxis(1) * .3); 
LimelightCam.visionOn(&stick); 
LimelightCam.visionMove(&drive, &stick); 
//ArmMotor.Set(stick.GetY()*0.1); 
// if(stick.GetRawButton(3)){ 
//   gearShifts.Set(frc::DoubleSolenoid::Value::kOff); 
// }
// else if(stick.GetRawButton(4)){ 
//   gearShifts.Set(frc::DoubleSolenoid::Value::kOff); 
// }
// if(xBox.GetPOV(0) < 0.1){ 
//   talon_foot->Set(ControlMode::PercentOutput, -.3); 
//   frc::SmartDashboard::PutNumber("POVLoop", 1); 
// } else if(xBox.GetPOV(0) > 179.0){ 
//   frc::SmartDashboard::PutNumber("POVLoop", 2); 
//   talon_foot->Set(ControlMode::PercentOutput, .3); 
// } else { 
//   frc::SmartDashboard::PutNumber("POVLoop", 3); 
//   talon_foot->Set(ControlMode::PercentOutput, -.05);
// }
if(!xBox.GetRawButton(4) && !stick.GetRawButton(4)){
  if(fabs(stick.GetRawButton(6))){
  talon_foot->Set(ControlMode::PercentOutput, 1);
  } 
  else { 
  talon_foot->Set(ControlMode::PercentOutput, .2);
  }
}
else if(xBox.GetRawButton(4) || stick.GetRawButton(4)){ 
  double fkp = 1.4; 
 double targetPitch = -11; 
  double pitchError = targetPitch - myAhrs->GetPitch(); 
  double targetPitchFull = -7; 
  if(myAhrs->GetPitch() > targetPitchFull){ 
  talon_foot->Set(ControlMode::PercentOutput, -1); 

  }
  else{ 
  talon_foot->Set(ControlMode::PercentOutput, fkp* pitchError); 
  frc::SmartDashboard::PutNumber("Measured Pitch Output", fkp*pitchError); 

  }
}
frc::SmartDashboard::PutNumber("Talon foot output", talon_foot->GetMotorOutputPercent()); 
frc::SmartDashboard::PutNumber("Measured Pitch", myAhrs->GetPitch()); 

auxControl.intakeControl(&xBox, &intake); 
auxControl.moveArm(&xBox, &armPID, &armControlPID, &ArmMotor, myTimer->Get()); 
if(xBox.GetRawAxis(3) > 0.1){
  winchChannel.Set(xBox.GetRawAxis(3) * .5); 
} else if(xBox.GetRawAxis(2) > 0.1){ 
  winchChannel.Set(-xBox.GetRawAxis(2) * .5); 
} else {
  winchChannel.Set(0.0);
}
if(stick.GetRawButton(5)) { 
    gearShifts.Set(frc::DoubleSolenoid::Value::kForward); 
}
else{ 
  gearShifts.Set(frc::DoubleSolenoid::Value::kReverse); 
}
}


void Robot::TestPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <memory>
#include <iostream>
#include <string>

#include <frc/WPILib.h>
#include <Joystick.h>
#include <RobotDrive.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "AHRS.h"
#include "ctre/Phoenix.h"
#include "DigitalInput.h"
#include "DigitalOutput.h"
#include <rrScaler_V001.h>
#include <rrXbox_V001.h>
#include <rrConstants_V001.h>
//#include <rrPixy_V001.hpp>

class Robot : public frc::IterativeRobot {
public:

	WPI_TalonSRX leftChannel;
	WPI_TalonSRX rightChannel;
	WPI_TalonSRX leftFollow;
	WPI_TalonSRX rightFollow;
	WPI_TalonSRX winchChannel;
	WPI_TalonSRX winchFollow;
	WPI_TalonSRX leftGrabber;
	WPI_TalonSRX rightGrabber;
	frc::Joystick stick{0};
	frc::Joystick xbox{1};
	DifferentialDrive robotDrive;
//	neScaler scalerX;
	neScaler scalerY;
	neScaler scalerZ;
	neScaler scalerW;
	DoubleSolenoid shifter;
	//rrPixy Pixy;
	AHRS* Ahrs;
	Timer* mTimer;
	DigitalInput grabLimit;
	DigitalOutput liftUp{1};		//A2 bottom arduino
	DigitalOutput liftDown{0};		//A3 bottom arduino
	DigitalOutput intakeIn{4};		// top arduio
	DigitalOutput intakeOut{2};		// top arduino
	DigitalOutput intakeSwitch{3};		// top arduino

	Robot():
		//drive
		leftChannel(26),
		rightChannel(19),
		leftFollow(24),
		rightFollow(25),

		//auxiliary					//winch and grabber, etc.
		winchChannel(29),			//29 has the encoder
		winchFollow(28),
		leftGrabber(30),
		rightGrabber(31),

		//drive
		robotDrive(leftChannel, rightChannel),
		shifter(2,3),
		Ahrs(),
		mTimer(),
		grabLimit(0)


	{
		robotDrive.SetExpiration(.15);
		//Pixy.init(0x54);
	}

	int absPos = 0;

	void RobotInit() {
		winchChannel.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);
		winchChannel.ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, 0);

		rightFollow.Follow(rightChannel);
		leftFollow.Follow(leftChannel);
		winchFollow.Follow(winchChannel);
		Ahrs = new AHRS(SerialPort::kMXP);
		mTimer = new Timer;

//		autonChooser.AddDefault(autoSideMid, autoSideMid);
//		autonChooser.AddObject(autoSideLeft, autoSideLeft);
//		autonChooser.AddObject(autoSideRight, autoSideRight);
//		frc::SmartDashboard::PutData("Auto Start", &autonChooser);

		chooser.AddDefault(driveStickTurn, driveStickTurn);
		chooser.AddObject(driveStick, driveStick);
		chooser.AddObject(driveXbox, driveXbox);
		frc::SmartDashboard::PutData("Drive Modes", &chooser);

		winchChannel.SetSelectedSensorPosition(absPos, 0, 10);
//		absPos = winchChannel.GetSelectedSensorPosition(kPIDLoopIdx) & 0xFFF;
//
//
//		winchChannel.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, kPIDLoopIdx,
//				kTimeoutMs);
//
//		winchChannel.ConfigNominalOutputForward(0, kTimeoutMs);
//		winchChannel.ConfigNominalOutputReverse(0, kTimeoutMs);
//		winchChannel.ConfigPeakOutputForward(1, kTimeoutMs);
//		winchChannel.ConfigPeakOutputReverse(-1, kTimeoutMs);
//
//		winchChannel.ConfigClosedloopRamp(.5, kTimeoutMs);
//
//		winchChannel.Config_kP(0, kPVal, kTimeoutMs);
//		winchChannel.Config_kI(0, kIVal, kTimeoutMs);

//		exponent, slope, deadzone

//		scalerX.init(1.0, 1.0, 0.05);
		scalerY.init(2.0, 1.0, 0.05);
		scalerZ.init(4.0, 1.0, 0.05);
		scalerW.init(2.5, 1.0, 0.1);	//winch

		SmartDashboard::PutNumber("autoTest_Duration", 0.0);			//auto testing
		SmartDashboard::PutNumber("autoTest_Voltage", 0.0);

	}

	std::string gameData;
	int state = 0;

	void AutonomousInit() override {
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		SmartDashboard::PutString("Game Data", gameData);


		mTimer->Start();
		mTimer->Reset();

		Ahrs->Reset();

		state = 0;
		grabState = 0;
	}


//	void midLeftAuto (double speedOutput) {
//		switch(state){
//			case 0:{
//				if(mTimer->Get() > 3){
//					robotDrive.ArcadeDrive(0,0,0);
//					mTimer->Reset();
//					state++;
//				}
//				robotDrive.ArcadeDrive(speedOutput, 0, 0);
//				SmartDashboard::PutNumber("debugcase0", 0);
//				break;
//			}
//			case 1:{
//
//				SmartDashboard::PutNumber("debugcase1", 1);
//				if(Ahrs->GetAngle() > -90 && Ahrs->GetAngle() < -85){
//				robotDrive.ArcadeDrive(0,0,0);
//				mTimer->Reset();
//				state++;
//				}
//				if(Ahrs->GetAngle() > -90){
//					robotDrive.ArcadeDrive(0,-0.3,0);
//				} else if(Ahrs->GetAngle() < -90){
//					robotDrive.ArcadeDrive(0,0.3,0);
//				} else{
//					robotDrive.ArcadeDrive(0,0,0);
//					state++;
//					}
//
//				break;
//			}
//			case 2:{
//				if(mTimer->Get() > 1.5){
//					robotDrive.ArcadeDrive(0,0,0);
//					mTimer->Reset();
//					state++;
//				}
//				robotDrive.ArcadeDrive(speedOutput,0,0);
//				break;
//			}
//			case 3:{
//				if(Ahrs->GetAngle() > 2){
//					robotDrive.ArcadeDrive(0,-0.3,0);
//				} else if(Ahrs->GetAngle() < -3){
//					robotDrive.ArcadeDrive(0,0.3,0);
//				} else{
//					robotDrive.ArcadeDrive(0,0,0);
//					mTimer->Reset();
//					state++;
//				}
//				break;
//			}
//			case 4:{
//				if(mTimer->Get() > 4){
//					robotDrive.ArcadeDrive(0,0,0);
//					mTimer->Reset();
//					state++;
//				}
//				robotDrive.ArcadeDrive(speedOutput,0,0);
//				break;
//			}
//			case 5:{
//				if(Ahrs->GetAngle() > 95 && Ahrs->GetAngle() < 85){
//					robotDrive.ArcadeDrive(0,0,0);
//					mTimer->Reset();
//					state++;
//					}
//				if(Ahrs->GetAngle() > 90){
//					robotDrive.ArcadeDrive(0,-0.3,0);
//				} else if(Ahrs->GetAngle() < 90){
//					robotDrive.ArcadeDrive(0,0.3,0);
//				} else{
//					robotDrive.ArcadeDrive(0,0,0);
//					state++;
//				}
//				break;
//			}
//			case 6:{
//				if(mTimer->Get() >= 1.5){
//					robotDrive.ArcadeDrive(0,0,0);
//					mTimer->Reset();
//					state++;
//				}
//				robotDrive.ArcadeDrive(speedOutput,0,0);
//				break;
//			}
//
//			default:{
//				robotDrive.ArcadeDrive(0,0,0);
//			}
//		}
//	}

//	void midRightAuto(double speedOutput){
//
//			switch(state){
//				case 0:{
//					if(mTimer->Get() > 3){
//						robotDrive.ArcadeDrive(0,0,0);
//						mTimer->Reset();
//						state++;
//					}
//					robotDrive.ArcadeDrive(speedOutput, 0, 0);
//					SmartDashboard::PutNumber("debugcase0", 0);
//					break;
//				}
//				case 1:{
//
//					SmartDashboard::PutNumber("debugcase1", 1);
//					if(Ahrs->GetAngle() > 92){
//						robotDrive.ArcadeDrive(0,0.3,0);
//					} else if(Ahrs->GetAngle() < 89){
//						robotDrive.ArcadeDrive(0,-0.3,0);
//					} else{
//						robotDrive.ArcadeDrive(0,0,0);
//						mTimer->Reset();
//						state++;
//						}
//
//					break;
//				}
//				case 2:{
//					if(mTimer->Get() > 4.5){
//						robotDrive.ArcadeDrive(0,0,0);
//						mTimer->Reset();
//						state++;
//					}
//					robotDrive.ArcadeDrive(speedOutput,0,0);
//					break;
//				}
//				case 3:{
//					if(Ahrs->GetAngle() > 2){
//						robotDrive.ArcadeDrive(0,0.3,0);
//					} else if(Ahrs->GetAngle() < -3){
//						robotDrive.ArcadeDrive(0,-0.3,0);
//					} else{
//						robotDrive.ArcadeDrive(0,0,0);
//						mTimer->Reset();
//						state++;
//					}
//					break;
//				}
//				case 4:{
//					if(mTimer->Get() > 9){
//						robotDrive.ArcadeDrive(0,0,0);
//						mTimer->Reset();
//						state++;
//					}
//					robotDrive.ArcadeDrive(speedOutput,0,0);
//					break;
//				}
//				case 5:{
//
//					if(Ahrs->GetAngle() > -89){
//						robotDrive.ArcadeDrive(0,0.3,0);
//					} else if(Ahrs->GetAngle() < -92){
//						robotDrive.ArcadeDrive(0,-0.3,0);
//					} else{
//						robotDrive.ArcadeDrive(0,0,0);
//						mTimer->Reset();
//						state++;
//					}
//					break;
//				}
//				case 6:{
//					if(mTimer->Get() >= 1.5){
//						robotDrive.ArcadeDrive(0,0,0);
//						mTimer->Reset();
//						state++;
//					}
//					robotDrive.ArcadeDrive(speedOutput,0,0);
//					break;
//				}
//
//				default:{
//					robotDrive.ArcadeDrive(0,0,0);
//				}
//			}
//		}

//	void forwardAuto(double speedOutput){
//		switch(state){
//			case 0: {
//				if(mTimer->Get() >= 6.5){
//					robotDrive.ArcadeDrive(0,0,0);
//					mTimer->Reset();
//					state++;
//				}
//				robotDrive.ArcadeDrive(0.3,0,0);
//				break;
//			}
//			default:{
//				robotDrive.ArcadeDrive(0,0,0);
//				break;
//			}
//		}
//	}

	void autoTesting(){
		double autoTest_Duration = SmartDashboard::GetNumber("autoTest_Duration", 0.0);
		double autoTest_Voltage = SmartDashboard::GetNumber("autoTest_Voltage", 0.0);

		switch(state){
			case 0:{
				if(mTimer->Get() > autoTest_Duration){
					robotDrive.ArcadeDrive(0,0,0);
					mTimer->Reset();
					state++;
				}
				robotDrive.ArcadeDrive(autoTest_Voltage, 0, 0);
				SmartDashboard::PutNumber("debugcase0", 0);
				break;
			}
			case 1:{

				SmartDashboard::PutNumber("debugcase1", 1);
				if(Ahrs->GetAngle() > -90 && Ahrs->GetAngle() < -85){
				robotDrive.ArcadeDrive(0,0,0);
				mTimer->Reset();
				state++;
				}
				if(Ahrs->GetAngle() > -91){
					robotDrive.ArcadeDrive(0,-0.3,0);
				} else if(Ahrs->GetAngle() < -88){
					robotDrive.ArcadeDrive(0,0.3,0);
				} else{
					robotDrive.ArcadeDrive(0,0,0);
					mTimer->Reset();
					state++;
					}

				break;
			}
			case 2:{
				if(mTimer->Get() > autoTest_Duration){
					robotDrive.ArcadeDrive(0,0,0);
					mTimer->Reset();
					state++;
				}
				robotDrive.ArcadeDrive(autoTest_Voltage,0,0);
				break;
			}
			case 3:{
				if(Ahrs->GetAngle() > 2){
					robotDrive.ArcadeDrive(0,-0.3,0);
				} else if(Ahrs->GetAngle() < -3){
					robotDrive.ArcadeDrive(0,0.3,0);
				} else{
					robotDrive.ArcadeDrive(0,0,0);
					mTimer->Reset();
					state++;
				}
				break;
			}
			case 4:{
				if(mTimer->Get() > 4){
					robotDrive.ArcadeDrive(0,0,0);
					mTimer->Reset();
					state++;
				}
				robotDrive.ArcadeDrive(autoTest_Voltage,0,0);
				break;
			}

//			case 5:{
//				if(Ahrs->GetAngle() > 95 && Ahrs->GetAngle() < 85){
//					robotDrive.ArcadeDrive(0,0,0);
//					mTimer->Reset();
//					state++;
//					}
//				if(Ahrs->GetAngle() > 90){
//					robotDrive.ArcadeDrive(0,-0.3,0);
//				} else if(Ahrs->GetAngle() < 90){
//					robotDrive.ArcadeDrive(0,0.3,0);
//				} else{
//					robotDrive.ArcadeDrive(0,0,0);
//					state++;
//				}
//				break;
//			}


			default:{
				robotDrive.ArcadeDrive(0,0,0);
				break;
			}
	}
	}



	void AutonomousPeriodic() {

//		double speedOutput = 0;
//		speedOutput = SmartDashboard::GetNumber("Speed Output", 0.0);

		SmartDashboard::PutNumber("Gyro Angle", Ahrs->GetAngle());
		SmartDashboard::PutNumber("Current State", state);
		SmartDashboard::PutNumber("Timer", mTimer->Get());
		SmartDashboard::PutNumber("Debug Periodic", 2);


		robotDrive.ArcadeDrive(0,0);
		SmartDashboard::PutString("Selected Auto Cmd", "autoSideMid:Left");
		autoTesting();

//		autonSide = autonChooser.GetSelected();
//		std::cout << "Side Selected: " << autonSide;
//
//		if (autonSide == autoSideLeft){
//			if (gameData[0] == 'L'){
//				//forward, place right
//				SmartDashboard::PutString("Selected Auto Cmd", "autoSideLeft:PlaceRight");
//			} else {
//				//forward
//				forwardAuto(speedOutput);
//				SmartDashboard::PutString("Selected Auto Cmd", "autoSideLeft:Forward");
//			}
//		} else if (autonSide == autoSideMid){
//			if (gameData[0] == 'L'){
//				midLeftAuto(speedOutput);
//				SmartDashboard::PutString("Selected Auto Cmd", "autoSideMid:Left");
//			} else {
//				midRightAuto(speedOutput);
//				SmartDashboard::PutString("Selected Auto Cmd", "autoSideMid:Right");
//			}
//		} else
//			if (autonSide == autoSideRight){
//			if (gameData[0] == 'L'){
//				//forward
//				switch(state){
//					case 0: {
//						if(mTimer->Get() >= 6.5){
//							robotDrive.ArcadeDrive(0,0,0);
//							mTimer->Reset();
//							state++;
//						}
//						robotDrive.ArcadeDrive(0.3,0,0);
//						break;
//					}
//					default:{
//						robotDrive.ArcadeDrive(0,0,0);
//						break;
//					}
//				}
//				SmartDashboard::PutString("Selected Auto Cmd", "autoSideRight:Forward");
//			} else {
//				//forward place left
//			}
//		}

	}


	void TeleopInit() {

		grabState = 0;
		winchDriver = 0;
	}

//	int currentState = 0;
//	int nextState = 0;

	double winchDriver = 0;

//	double pdpWinch = 0;
//	double pdpWinchF = 0;
//	int pdpCompressor = 0;

	int winchClimbState = 0;
	int grabState = 0;
	int grabTime = 0;

	void TeleopPeriodic() {

		AaronLED();

//		pdpWinch = pdp.GetCurrent(15);				//winch primary motor
//		pdpWinchF = pdp.GetCurrent(14);				//winch follower motor
//		pdpCompressor = m_pdp.GetCurrent()

		SmartDashboard::PutNumber("JoystickY", stick.GetY());
		SmartDashboard::PutNumber("ScaledY", scalerY.scaleOutput(stick.GetY()));
		SmartDashboard::PutNumber("JoystickZ", stick.GetZ());

		SmartDashboard::PutNumber("Gyro Angle", Ahrs->GetAngle());

		SmartDashboard::PutNumber("Right Trigger Output", xbox.GetRawAxis(kRightTrigger));
		SmartDashboard::PutNumber("Left Trigger Output", xbox.GetRawAxis(kLeftTrigger));
		SmartDashboard::PutNumber("Right Trigger Scaled", scalerW.scaleOutput(xbox.GetRawAxis(kRightTrigger)));
		SmartDashboard::PutNumber("Left Trigger Scaled", scalerW.scaleOutput(xbox.GetRawAxis(kLeftTrigger)));

		SmartDashboard::PutNumber("Winch Encoder", winchChannel.GetSelectedSensorPosition(kPIDLoopIdx));
		SmartDashboard::PutNumber("Winch Drive Value", winchDriver);

//		SmartDashboard::PutNumber("PDP Amp; Winch Primary", pdpWinch);
//		SmartDashboard::PutNumber("PDP Amp; Winch Follower", pdpWinchF);

		driveSelected = chooser.GetSelected(); //driveStickTurn;
		std::cout << "Drive Selected: " << driveSelected;


		if (driveSelected == driveStickTurn) {
		robotDrive.ArcadeDrive(-1*(scalerY.scaleOutput(stick.GetY())),
			scalerZ.scaleOutput(stick.GetZ()));
		} else if (driveSelected == driveStick) {
		robotDrive.ArcadeDrive(-1*(scalerY.scaleOutput(stick.GetY())),
			scalerZ.scaleOutput(stick.GetX()));
		} else if (driveSelected == driveXbox) {
		robotDrive.ArcadeDrive(-1*(scalerY.scaleOutput(xbox.GetRawAxis(kLeftYAxis))),
			scalerZ.scaleOutput(xbox.GetRawAxis(kLeftXAxis)));
		}


		if (stick.GetRawButton(3)) {
			shifter.Set(frc::DoubleSolenoid::kForward);
			SmartDashboard::PutNumber("Gear Shift State", 1);
		} else if (stick.GetRawButton(4)) {
			shifter.Set(frc::DoubleSolenoid::kReverse);
			SmartDashboard::PutNumber("Gear Shift State", 0);
		}

		if (xbox.GetRawButton(kXboxLeftB)){
		winchChannel.SetSelectedSensorPosition(absPos, 0, 10);
		}

//		if (xbox.GetRawAxis(kRightTrigger) > 0) {
//				winchChannel.Set(scalerW.scaleOutput(xbox.GetRawAxis(kRightTrigger)));
//			} else if (xbox.GetRawAxis(kLeftTrigger) > 0) {
//				winchChannel.Set(-1*(scalerW.scaleOutput(xbox.GetRawAxis(kLeftTrigger))));
//			} else if ((xbox.GetRawAxis(kLeftTrigger) == 0) && (xbox.GetRawAxis(kRightTrigger) == 0)){
//				winchChannel.Set(0);
//			}

		switch(winchClimbState){
			case 0: {  // Init
				if (xbox.GetRawAxis(kRightTrigger) > 0) {
					winchDriver = scalerW.scaleOutput(xbox.GetRawAxis(kRightTrigger)); 					//winchChannel.Set(scalerW.scaleOutput(xbox.GetRawAxis(kRightTrigger)));
				} else if (xbox.GetRawAxis(kLeftTrigger) > 0) {
					winchDriver = -1*(scalerW.scaleOutput(xbox.GetRawAxis(kLeftTrigger)));		    	//winchChannel.Set(-1*(scalerW.scaleOutput(xbox.GetRawAxis(kLeftTrigger))));
				} else if ((xbox.GetRawAxis(kLeftTrigger) == 0) && (xbox.GetRawAxis(kRightTrigger) == 0)){
					winchDriver = 0; //winchChannel.Set(0);
				}
				if(xbox.GetRawButton(kXboxRightB)){
//					winchClimbState++;
				}
				break;

			}
			case 1: {
				if((winchChannel.GetSelectedSensorPosition(kPIDLoopIdx) < -6000) && xbox.GetRawButton(kXboxRightB)) {
					winchClimbState++;
				}
				winchDriver = .6; //winchChannel.Set(-.6);
				break;
			}
			case 2: {
				if(winchChannel.GetSelectedSensorPosition(kPIDLoopIdx) < -8000) {
					winchClimbState++;
				}
				winchDriver = .4; //winchChannel.Set(-.4);
				break;
			}
			case 3: {
				if(winchChannel.GetSelectedSensorPosition(kPIDLoopIdx) <= -10000){
					winchClimbState = 0;
				}
				winchDriver = .3; //winchChannel.Set(-.3);
				break;
			}
		}

		winchChannel.Set(winchDriver);



//		winchChannel.EnableCurrentLimit(true);
//		winchChannel.ConfigPeakCurrentLimit(35, 10); /* 35 A */
//		winchChannel.ConfigPeakCurrentDuration(200, 10); /* 200ms */
//		winchChannel.ConfigContinuousCurrentLimit(30, 10); /* 30A */

//		if (xbox.GetRawButton(kXboxLThumb) == 1){
//			grabState = 0;
//		}
//
//		switch(grabState) {
//			case 0:
//				leftGrabber.Set(0);
//				rightGrabber.Set(0);
//				if (xbox.GetRawButton(kXboxA) == 1){
//					grabState++;
//				}
//				break;
//			case 1:
//				leftGrabber.Set(-1);
//				rightGrabber.Set(1);
//				if (grabLimit.Get() == 1){
//					leftGrabber.Set(-.2);
//					rightGrabber.Set(.2);
//				} else if (grabLimit.Get() == 0){
//					leftGrabber.Set(-1);
//					rightGrabber.Set(1);
//				}
//				if (xbox.GetRawButton(kXboxB)){
//					grabState = 2;
//				}
//				break;
//			case 2:
//				grabTime = grabTime + 1;
//				grabState = 3;
//				break;
//			case 3:
//				leftGrabber.Set(1);
//				rightGrabber.Set(-1);
//				if (grabTime >= 350){
//					grabState= 0;
//				}
//				break;
//		}
		SmartDashboard::PutNumber("Grabber Switch", grabLimit.Get());
		SmartDashboard::PutNumber("Grabber State", grabState);
//

		if (xbox.GetRawButton(kXboxB)) {		//push out
			leftGrabber.Set(1);
			rightGrabber.Set(-1);
		} else if (xbox.GetRawButton(kXboxA)) { //intake
			leftGrabber.Set(-1);
			rightGrabber.Set(1);
		} else if (xbox.GetRawButton(kXboxX)) {
			leftGrabber.Set(-.2);
			rightGrabber.Set(.2);
		} else {
			leftGrabber.Set(0);
			rightGrabber.Set(0);
		}
	}

	void TestPeriodic() {

	}

	void AaronLED() {
		if (xbox.GetRawAxis(kLeftTrigger) > 0) {
			liftDown.Set(false);
		}
		else if (xbox.GetRawAxis(kRightTrigger) > 0) {
			liftDown.Set(true);
		}
//		else {
//			liftUp.Set(false);
//			liftDown.Set(false);
//		}

		if (xbox.GetRawButton(kXboxA)) {
			intakeIn.Set(true);
		}
		else if (xbox.GetRawButton(kXboxB)) {
			intakeOut.Set(true);
		}
		else {
			intakeIn.Set(false);
			intakeOut.Set(false);
		}
		if (grabLimit.Get()) {
			intakeSwitch.Set(true);
		}
		else{
			intakeSwitch.Set(false);
		}

	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();

	frc::SendableChooser<std::string> chooser;
	const std::string driveStickTurn = "Joystick Twist";
	const std::string driveXbox = "Xbox Drive";
	const std::string driveStick = "Joystick Drive";
	std::string driveSelected;

	frc::SendableChooser<std::string> autonChooser;
	const std::string autoSideMid = "Middle";
	const std::string autoSideLeft = "Left";
	const std::string autoSideRight = "Right";
	std::string autonSide;

//	frc::PowerDistributionPanel pdp;

};

START_ROBOT_CLASS(Robot)

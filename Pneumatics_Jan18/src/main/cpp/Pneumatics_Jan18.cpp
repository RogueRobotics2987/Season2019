//////////////////////////////////////////////////////////////////////////////////////
//								Reece Torbert 2018									//
//////////////////////////////////////////////////////////////////////////////////////


#include <DoubleSolenoid.h>
#include <IterativeRobot.h>
#include <Joystick.h>
#include <Solenoid.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Timer.h>
#include "ctre/Phoenix.h"
#include <Drive/DifferentialDrive.h>
#include "AHRS.h"
#include <SmartDashboard/SendableChooser.h>
#include "WPILib.h"
#include "rrPixy.hpp"
#include "I2C.h"
#include <Drive/MecanumDrive.h>

//#include


class Robot : public frc::IterativeRobot {

public:

	RobotDrive robotDrive;
	frc::Joystick stick{0};
	rrPixy myPixy;

//	WPI_TalonSRX frontLeft;
//	WPI_TalonSRX frontRight;
//	WPI_TalonSRX rearLeft;
//	WPI_TalonSRX rearRight;


	//new robot
	WPI_TalonSRX rightChannel;
	WPI_TalonSRX rightFollow;
	WPI_TalonSRX leftChannel;
	WPI_TalonSRX leftFollow;


	Robot(): // @suppress("Class members should be properly initialized")

		//mittens
//		frontLeft(21),
//		frontRight(22),
//		rearLeft(27),
//		rearRight(20),
//		robotDrive(frontLeft, rearLeft, frontRight, rearRight)

		//new robot
		rightChannel(26),
		rightFollow(24),
		leftChannel(25),
		leftFollow(19),
		DifferentialDrive(rightChannel, leftChannel)


	{
		robotDrive.SetExpiration(.15);
		myPixy.init(0x54);
	}

	void TeleopPeriodic() override {

		robotDrive.ArcadeDrive(stick.GetX(), stick.GetY());
//		robotDrive.MecanumDrive_Cartesian(stick.GetX(),
//						stick.GetY(),
//						stick.GetZ());

//		m_solenoid.Set(m_stick.GetRawButton(kSolenoidButton));


		if (stick.GetRawButton(3)) {
			m_doubleSolenoid.Set(frc::DoubleSolenoid::kForward);
			SmartDashboard::PutNumber("Solenoid Engaged", 1);
		} else if (stick.GetRawButton(4)) {
			m_doubleSolenoid.Set(frc::DoubleSolenoid::kReverse);
			SmartDashboard::PutNumber("Solenoid Engaged", 0);
		} else {
			m_doubleSolenoid.Set(frc::DoubleSolenoid::kOff);
		}
		Wait(.005);
	}

	uint16_t numBlocks = 0;
	uint16_t lastX;
	uint16_t lastY;
//	uint16_t

	void AutonomousPeriodic() {
			robotDrive.MecanumDrive_Cartesian(0,0,0);

		numBlocks = myPixy.rrPixyGetBlocks(100);
//		if((numBlocks ==1) && (myPixy.rrReturnBlock0().signature==1)){
					lastX = myPixy.rrReturnBlock0().x;
					lastY = myPixy.rrReturnBlock0().y;
					SmartDashboard::PutNumber("Block @ X = ",lastX);
					SmartDashboard::PutNumber("Block @ Y = ",lastY);
					SmartDashboard::PutNumber("# of Blocks = ", numBlocks);
//		}
	}

	void RobotInit() {
		rightFollow.Follow(rightChannel);
		leftFollow.Follow(leftChannel);

		//channel inversions
//		rightChannel.SetInverted(false)
//		rightFollow.SetInverted(false)
//		leftChannel.SetInverted(false)
//		leftFollow.SetInverted(false)

	}


private:


//	frc::Joystick m_stick{0};

//  Solenoid corresponds to a single solenoid.
//	frc::Solenoid m_solenoid{0};

	// DoubleSolenoid corresponds to a double solenoid.
	frc::DoubleSolenoid m_doubleSolenoid{ 2, 3 };

//	static constexpr int kSolenoidButton = 5;
//	static constexpr int kDoubleSolenoidForward = 3;
//	static constexpr int kDoubleSolenoidReverse = 4;
};

START_ROBOT_CLASS(Robot)

#ifndef xBoxControl_h 
#define xBoxControl_h 
#include <frc/Wpilib.h>
#include "rev/CANSparkMax.h"
#include <ctre/Phoenix.h> 
#include "PIDControl.h"
#include <frc/smartdashboard/SmartDashboard.h>

class xBoxController{ 
    public: 
    double winchPos =  frc::SmartDashboard::PutNumber("Winch Position", 0); 

    double maxSpeed = 36; 
    
    xBoxController(); 
    void setShooter(frc::Joystick* xBox, frc::Solenoid* shooter); 
    void moveWinch(frc::Joystick* xBox, rev::CANSparkMax* winchMotor, rev::CANPIDController* winchPID, PIDControl* winchControlPID, double myTime); 
    void intakeControl(frc::Joystick* xBox, WPI_TalonSRX* intakeMotor);
    void MoveWinchNoStick(rev::CANPIDController* winchPID); 
    void moveArm(frc::Joystick* xBox, rev::CANPIDController* armPID, PIDControl* armControlPID, rev::CANSparkMax* armMotor, double myTime);
    void consecArmWinch(frc::Joystick* xBox, rev::CANPIDController* armPID, PIDControl* armControlPID, rev::CANSparkMax* armMotor, rev::CANSparkMax* winchMotor, double myTime);
    }; 
#endif xBoxControl_h 
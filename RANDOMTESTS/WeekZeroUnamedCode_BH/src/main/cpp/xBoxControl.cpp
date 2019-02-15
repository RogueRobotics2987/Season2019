#include "xBoxControl.h" 
xBoxController::xBoxController(){}
void xBoxController::setShooter(frc::Joystick* xBox, frc::Solenoid* shooter){ 
    if(xBox->GetRawButton(1)){ 
    shooter->Set(true); 
  }
  else if(!xBox->GetRawButton(1)){ 
    shooter->Set(false); 
  }
}

void xBoxController::moveWinch(frc::Joystick* xBox, rev::CANSparkMax*
                                 winchMotor, rev::CANPIDController* winchPID, 
                                 PIDControl* winchMotorPID, double myTime){ 
    if(xBox->GetRawAxis(3) > 0.1){
        winchPos = winchMotorPID->getNewPosStick( maxSpeed*xBox->GetRawAxis(3), myTime);
    }
    else if(xBox->GetRawAxis(2) > 0.1){ 
        winchPos = winchMotorPID->getNewPosStick(-maxSpeed*xBox->GetRawAxis(2), myTime);
    }

    else
    {
        winchPos = winchMotorPID->getNewPosStick(0.0, myTime);

    }

    winchPID->SetReference(winchPos, rev::ControlType::kPosition); 
    frc::SmartDashboard::PutNumber("Winch PID reference", winchPos);
}
void xBoxController::intakeControl(frc::Joystick* xBox, WPI_TalonSRX* intakeMotor){ 
    if(xBox->GetRawButton(1)){ 
    intakeMotor->Set(.2); 
  }
  else if(xBox->GetRawButton(2)){ 
    intakeMotor->Set(-.2); 
  }
  else{ 
      intakeMotor->Set(0.0); 
  }
 
}
 void xBoxController::MoveWinchNoStick(rev::CANPIDController* winchPID){ 
    winchPos = frc::SmartDashboard::GetNumber("Winch Reference", winchPos); 

    winchPID->SetReference(winchPos, rev::ControlType::kPosition);

  }
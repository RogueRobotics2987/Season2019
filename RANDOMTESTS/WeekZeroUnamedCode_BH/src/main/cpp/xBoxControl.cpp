#include "xBoxControl.h" 
xBoxController::xBoxController(){}
void xBoxController::setShooter(frc::Joystick* xBox, frc::Solenoid* shooter){ 
    if(xBox->GetRawButton(3)){ 
    shooter->Set(true); 
  }
  else if(!xBox->GetRawButton(4)){ 
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
    intakeMotor->Set(.3); 
  }
  else if(xBox->GetRawButton(2)){ 
    intakeMotor->Set(-.3); 
  }
  else{ 
      intakeMotor->Set(0.0); 
  }
 
}
 void xBoxController::MoveWinchNoStick(rev::CANPIDController* winchPID){ 
    winchPos = frc::SmartDashboard::GetNumber("Winch Reference", winchPos); 

    winchPID->SetReference(winchPos, rev::ControlType::kPosition);

  }
    void xBoxController::moveArm(frc::Joystick* xBox, rev::CANPIDController* armPID, PIDControl* armControlPID, rev::CANSparkMax* armMotor, double myTime){
        rev::CANEncoder armEncoder = armMotor->GetEncoder(); 
        if(armEncoder.GetPosition() > 84.0){ 
        armPID->SetReference(84, rev::ControlType::kPosition);
        }
        else if(armEncoder.GetPosition() <0){ 
            armPID->SetReference(0, rev::ControlType::kPosition); 
        }
        else{ 
        armPID->SetReference(armControlPID->getNewPosStick(xBox->GetRawAxis(1) * 14, myTime), rev::ControlType::kPosition); 
        } 
          frc::SmartDashboard::PutNumber("Arm Reference", armControlPID->getNewPosStick(xBox->GetRawAxis(1) * 7, myTime));


    }
    // 82.1 Up 0 Down 
 void xBoxController::consecArmWinch(frc::Joystick* xBox, rev::CANPIDController* armPID, PIDControl* armControlPID, rev::CANSparkMax* armMotor, rev::CANSparkMax* winchMotor, double myTime){ 
    rev::CANEncoder winchEncoder = winchMotor->GetEncoder(); 
    rev::CANEncoder armEncoder = armMotor->GetEncoder(); 
    rev::CANPIDController winchPID = armMotor->GetPIDController(); 
    
        if((winchEncoder.GetPosition() > 80.0) && (xBox->GetRawAxis(3) > .1)){ 
            
           armPID->SetReference(xBox->GetRawAxis(3), rev::ControlType::kPosition);
           winchMotor->Set(0.14); 
        
        }
        else if(winchEncoder.GetPosition() > 80 && xBox->GetRawAxis(2) > .1 ) { 
            armPID->SetReference(-xBox->GetRawAxis(2), rev::ControlType::kPosition);
            winchMotor->Set(0.14); 
        }

        else if(winchEncoder.GetPosition() < 80){ 

            if(xBox->GetRawAxis(3) > 0.1){
            winchMotor->Set(xBox->GetRawAxis(3)); 
            } else if(xBox->GetRawAxis(2) > 0.1){ 
            winchMotor->Set(-xBox->GetRawAxis(2)); 
            } else {
            winchMotor->Set(0.0);
            }

        }
}



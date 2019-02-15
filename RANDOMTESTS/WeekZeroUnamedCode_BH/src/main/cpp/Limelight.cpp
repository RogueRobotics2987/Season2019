#include "Limelight.h"

LimelightControl::LimelightControl(){}
void LimelightControl::visionMove(frc::DifferentialDrive* drive, frc::Joystick* stick){
     targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
targetArea = table->GetNumber("ta",0.0);
targetSkew = table->GetNumber("ts",0.0);
deadZone = .2;
curTY=table->GetNumber("ty",0.0);
curTX=table->GetNumber("tx",0.0);
curNumTarget=table->GetNumber("tv",0.0);
HatchOffset = frc::SmartDashboard::GetNumber("Hatch Offset", HatchOffset);

if ((curNumTarget>=1)&&(stick->GetRawButton(1))&&(stick->GetRawButton(2))&&(stick->GetRawButton(7)))
{
drive->ArcadeDrive(-0.6/7.5*(curTY),0.02*curTX);
}
else if ((curNumTarget>=1)&&(stick->GetRawButton(1))&&(stick->GetRawButton(2))&&(stick->GetRawButton(8)))
{
 drive->ArcadeDrive(-0.6/7.5*(curTY-HatchOffset),0.02*curTX);
}
else {
drive->ArcadeDrive(stick->GetY(), stick->GetZ());

}

}
void LimelightControl::visionOn(frc::Joystick* stick){
   if(stick->GetRawButton(2)){
 frc::SmartDashboard::PutBoolean("VisionProcessor", true);
 // table->PutNumber("ledMode", 3);
 table->PutNumber("camMode", 0);
}
else {
 // table->PutNumber("ledMode", 1);
 table->PutNumber("camMode", 1);
 frc::SmartDashboard::PutBoolean("VisionProcessor", false);
}
if(stick->GetRawButton(1)){
 // table->PutNumber("stream", 1);
 table->PutNumber("ledMode", 3);
}
else {
 // table->PutNumber("stream", 0);
 table->PutNumber("ledMode", 1);
}
}

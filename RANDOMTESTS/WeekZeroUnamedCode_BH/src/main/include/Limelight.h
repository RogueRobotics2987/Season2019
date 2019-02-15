#ifndef Limelight_h
#define Limelight_h
#include <frc/smartdashboard/SmartDashboard.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Joystick.h>
#include <frc/WPIlib.h>
#include "ctre/Phoenix.h"

class LimelightControl{
   public:
   LimelightControl();
 
   std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr");
   double targetOffsetAngle_Horizontal = 0;
   double targetOffsetAngle_Vertical = 0;
   double targetArea = 0;
   double targetSkew = 0;
   double deadZone = .2;
   double curTY= 0;
   double curTX= 0;
   double curNumTarget=0;
   double HatchOffset = 0;

  void visionOn(frc::Joystick* stick);
  void visionMove(frc::DifferentialDrive* drive, frc::Joystick* stick);
};

#endif Limelight_h


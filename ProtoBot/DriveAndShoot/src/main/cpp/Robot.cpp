#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

const static double kToleranceDegrees = 2.0f;
const static double kMaxRotateRate = 0.5;
const static double kGamepadDeadZone = 0.1;
const static double kSlowSpeedFactor = 0.5;
const static double kFastSpeedFactor = 1.0;

const static double kPtuned = 0.006;
const static double kItuned = 0.0015;
const static double kDtuned = 0.001;

void Robot::RobotInit() {

  m_stick = new Joystick(joystickChannel);
	rotateToAngleRate = 0.0f;

  m_robotDrive.SetExpiration(0.1);

  try
  {
    /***********************************************************************
     * navX-MXP:
     * - Communication via RoboRIO MXP (SPI, I2C) and USB.            
     * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
     * 
     * navX-Micro:
     * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
     * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
     * 
     * VMX-pi:
     * - Communication via USB.
     * - See https://vmx-pi.kauailabs.com/installation/roborio-installation/
     * 
     * Multiple navX-model devices on a single robot are supported.
     ************************************************************************/
    ahrs = new AHRS(SPI::Port::kMXP);
  }
  catch (std::exception &ex)
  {
    std::string what_string = ex.what();
    std::string err_msg("Error instantiating navX MXP:  " + what_string);
    const char *p_err_msg = err_msg.c_str();
    DriverStation::ReportError(p_err_msg);
  }

  /* this is used to tune the PID numbers
  frc::SmartDashboard::PutNumber("kP", kP);
  frc::SmartDashboard::PutNumber("kI", kI);
  frc::SmartDashboard::PutNumber("kD", kD);
  frc::SmartDashboard::PutNumber("MaxRotateRate", MaxRotateRate);
  */

}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
    m_timer.Reset();
    m_timer.Start();
}

void Robot::AutonomousPeriodic() {
    // simple motion to validate motor configuration
    // Drive for 2 seconds
    if (m_timer.Get() < 2.0) {
      m_robotDrive.TankDrive(0.5, 0); // left motor only
    } else if (m_timer.Get() < 4.0) {
      m_robotDrive.TankDrive(0, 0.5); // right motor only
    } else if (m_timer.Get() < 6.0) {
      // Drive forwards half speed
      m_robotDrive.ArcadeDrive(0.5, 0.0);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0);
    }

}

void Robot::TeleopInit() {
  ahrs->ZeroYaw();
  kP = kPtuned;
  kI = kItuned;
  kD = kDtuned;
  /* used to tune PID numbers
  kP = frc::SmartDashboard::GetNumber("kP", kP);
  kI = frc::SmartDashboard::GetNumber("kI", kI);
  kD = frc::SmartDashboard::GetNumber("kD", kD);
  MaxRotateRate = frc::SmartDashboard::GetNumber("MaxRotateRate", MaxRotateRate);
  */
  m_pidController = new frc2::PIDController (kP, kI, kD);
  m_pidController->SetTolerance(8, 8);  // within 8 degrees of target is considered on set point
}

double Robot::TrimSpeed (double s, double max) {
  double result = s > max ? max : s;
  result = result < -max ? -max : result;
  return result;
}

double Robot::ScaleSpeed (double s, double scale) {
  return s * scale;
}

double Robot::ConvertRadsToDegrees (double rads) {
  const static double conversion_factor = 180.0/3.141592653589793238463;
  return rads * conversion_factor;
}

void Robot::TeleopPeriodic() {
  
  bool reset_yaw_button_pressed = m_stick->GetRawButton(1);
  if ( reset_yaw_button_pressed ) {
      ahrs->ZeroYaw();
  }

  frc::SmartDashboard::PutNumber("Angle", ahrs->GetAngle());

  bool rotateToAngle = false;
  //bool stepOver = false;
  if ( m_stick->GetPOV() == 0) {
      m_pidController->SetSetpoint(0.0f);
      rotateToAngle = true;
  } else if ( m_stick->GetPOV() == 90) {
      m_pidController->SetSetpoint(90.0f);
      rotateToAngle = true;
  } else if ( m_stick->GetPOV() == 180) {
      m_pidController->SetSetpoint(179.9f);
      rotateToAngle = true;
  } else if ( m_stick->GetPOV() == 270) {
      m_pidController->SetSetpoint(-90.0f);
      rotateToAngle = true;
  // }  else if ( stick->GetRawButton(1)) {
  //     m_pidController->SetSetpoint(-90.0f);
  //     stepOver = true;
  // }  else if ( stick->GetRawButton(3)) {
  //     m_pidController->SetSetpoint(90.0f);
  //     stepOver = true;
  }

  double field_rel_X = m_stick->GetRawAxis(2);
  double field_rel_Y = -m_stick->GetRawAxis(3);
  if (abs(field_rel_X) > kGamepadDeadZone || abs(m_stick->GetRawAxis(3)) > kGamepadDeadZone) {
    rotateToAngle = true;
    double angle =  ConvertRadsToDegrees(atan(field_rel_Y/field_rel_X));
    m_pidController->SetSetpoint(angle);
  }
  frc::SmartDashboard::PutNumber ("Angle set point", m_pidController->GetSetpoint());
  frc::SmartDashboard::PutNumber ("X", field_rel_X);
  frc::SmartDashboard::PutNumber ("Y", field_rel_Y);

  rotateToAngleRate = m_pidController->Calculate(ahrs->GetAngle());
  // trim the speed so it's not too fast
  rotateToAngleRate = TrimSpeed(rotateToAngleRate, kMaxRotateRate);

  bool slow_gear_button_pressed = m_stick->GetRawButton(5);
  bool high_gear_button_presssed = m_stick->GetRawButton(7);
  if (slow_gear_button_pressed) {speed_factor = kSlowSpeedFactor;}
  else if (high_gear_button_presssed) {speed_factor = kFastSpeedFactor;}
  frc::SmartDashboard::PutNumber ("Drive Speed Factor", speed_factor);

  try {

    if (rotateToAngle) {
      // MJS: since it's diff drive instead of mecanum drive, use tank method for rotation
      frc::SmartDashboard::PutNumber("rotateToAngleRate", rotateToAngleRate);
      m_robotDrive.TankDrive(rotateToAngleRate, -rotateToAngleRate, false);
    // } else if (stepOver) {
    //   frc::SmartDashboard::PutNumber("rotateToAngleRate", rotateToAngleRate);
    //   if (m_pidController->AtSetpoint()) {
    //     m_robotDrive.TankDrive(MaxRotateRate, MaxRotateRate, false); // drive forward
    //   } else {
    //     m_robotDrive.TankDrive(rotateToAngleRate, -rotateToAngleRate, false);
    //   }
    } else {
      // not rotating; drive by stick
      m_robotDrive.ArcadeDrive(ScaleSpeed(-m_stick->GetY(), speed_factor), ScaleSpeed(m_stick->GetX(), speed_factor));
      m_pidController->Reset(); // clears out integral state, etc
    }
  } catch (std::exception& ex ) {
    std::string err_string = "Error communicating with Drive System:  ";
    err_string += ex.what();
    DriverStation::ReportError(err_string.c_str());
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

const static double kToleranceDegrees = 2.0f;
const static double kMaxRotateRate = 0.5;
const static double kGamepadDeadZone = 0.15;
const static double kSlowSpeedFactor = 0.5;
const static double kFastSpeedFactor = 1.0;

const static double kPtuned = 0.006;
const static double kItuned = 0.0015;
const static double kDtuned = 0.001;

void Robot::RobotInit() {

    /************* shooter configuration **************/
		m_shooter_star->ConfigFactoryDefault();
		m_shooter_port->ConfigFactoryDefault();

				/* set up followers */
		m_shooter_port->Follow(*m_shooter_star);
    m_shooter_star->SetInverted(true);
    m_shooter_port->SetInverted(false);
		
		// breaking mode
		m_shooter_star->SetNeutralMode(NeutralMode::Coast);
		m_shooter_port->SetNeutralMode(NeutralMode::Coast);

    /* first choose the sensor */
		m_shooter_star->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
		// phase for TalonFX integrated sensor is automatically correct

		/* set the peak and nominal outputs */
		m_shooter_star->ConfigNominalOutputForward(0, kTimeoutMs);
		m_shooter_star->ConfigNominalOutputReverse(0, kTimeoutMs);
		m_shooter_star->ConfigPeakOutputForward(1, kTimeoutMs);
		m_shooter_star->ConfigPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 */
		m_shooter_star->Config_kF(kPIDLoopIdx, 0.1097, kTimeoutMs);
		m_shooter_star->Config_kP(kPIDLoopIdx, 0.22, kTimeoutMs);
		m_shooter_star->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_shooter_star->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

    /************** other motor setup *****************/

    // Conveyers
    m_vert_conveyer.ConfigFactoryDefault();
    m_hor_conveyer.ConfigFactoryDefault();
    m_hor_conveyer.Follow(m_vert_conveyer);
		m_vert_conveyer.ConfigNominalOutputForward(0, kTimeoutMs);
		m_vert_conveyer.ConfigNominalOutputReverse(0, kTimeoutMs);
		m_vert_conveyer.ConfigPeakOutputForward(0.5, kTimeoutMs);
		m_vert_conveyer.ConfigPeakOutputReverse(-0.5, kTimeoutMs);

  m_stick = new Joystick(k_joystick_pilot);
  m_stick_copilot = new Joystick(k_joystick_copilot);
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
  
  /**************** buttons ******************/

  bool reset_yaw_button_pressed = m_stick->GetRawButton(1);
  bool rotateToAngle = false;
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
  }

  if ( reset_yaw_button_pressed ) {
      ahrs->ZeroYaw();
  }

  /******************** joysticks **************************/

  double field_rel_X = m_stick->GetRawAxis(2);
  double field_rel_Y = -m_stick->GetRawAxis(3);
  double conveyer_Y = m_stick_copilot->GetRawAxis(3);
  double shooter_X = m_stick_copilot->GetRawAxis(0);
  double shooter_Y = m_stick_copilot->GetRawAxis(1);
  double shooter_R = sqrt(shooter_X*shooter_X + shooter_Y*shooter_Y);

  frc::SmartDashboard::PutNumber("Angle", ahrs->GetAngle());
  frc::SmartDashboard::PutNumber("Shooter Magnitude", shooter_R);

  /****************** power stuff *******************/

  // shooter
  if (shooter_R > 0.1) {
   double targetVelocity_UnitsPer100ms = -1 * shooter_R * 1500.0 * 2048 / 600;
    m_shooter_star->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms);
   frc::SmartDashboard::PutNumber("shooter target", targetVelocity_UnitsPer100ms);
  } else {
    m_shooter_star->Set(ControlMode::Velocity, 0.0);
  }

  // power conveyer, which does not have encoders
  frc::SmartDashboard::PutNumber("conveyer power", conveyer_Y);
  m_vert_conveyer.Set(conveyer_Y);

  if (abs(field_rel_X) > kGamepadDeadZone || abs(m_stick->GetRawAxis(3)) > kGamepadDeadZone) {
    rotateToAngle = true;
    double angle =  90 - ConvertRadsToDegrees(atan(field_rel_Y/abs(field_rel_X)));
    angle = copysign(angle, field_rel_X); // make angle negative if X is negative
    m_pidController->SetSetpoint(angle);
  }
  frc::SmartDashboard::PutNumber ("Angle set point", m_pidController->GetSetpoint());
  frc::SmartDashboard::PutNumber ("X", field_rel_X);
  frc::SmartDashboard::PutNumber ("Y", field_rel_Y);

  rotateToAngleRate = m_pidController->Calculate(ahrs->GetAngle());
  // trim the speed so it's not too fast
  rotateToAngleRate = TrimSpeed(rotateToAngleRate, kMaxRotateRate);

  // high and low speed scaling
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

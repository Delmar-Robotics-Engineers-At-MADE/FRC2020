
#include <iostream>
#include <string>

#include <frc/TimedRobot.h>
#include <frc/DigitalInput.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>


#include "ctre/Phoenix.h"
#include "AHRS.h"

#include "Constants.h"

using namespace frc;

    // constants

    const static int k_joystick_pilot = 0;
    const static int k_joystick_copilot = 1;
	
	const static double kToleranceDegrees = 2.0f;
	const static double kMaxRotateRate = 0.5;
	const static double kGamepadDeadZone = 0.15;
	const static double kSlowSpeedFactor = 0.5;
	const static double kFastSpeedFactor = 1.0;
	


	double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
	const static double kPtuned = 0.006;
	const static double kItuned = 0.0015;
	const static double kDtuned = 0.001;

class Robot: public TimedRobot {

	// shooter 
	TalonFX * m_shooter_star = new TalonFX(15); // 15 is starboard, 0 is port
	TalonFX * m_shooter_port = new TalonFX(0); // 15 is starboard, 0 is port
	//Joystick * _joy = new Joystick(0);
	// std::string _sb;
	// int _loops = 0;

	// drive motors
    WPI_TalonSRX m_leftfront{1};
    WPI_TalonSRX m_leftrear{2};
    WPI_TalonSRX m_rightfront{13}; 
    WPI_TalonSRX m_rightrear{14};
    frc::SpeedControllerGroup m_left{m_leftfront, m_leftrear};
    frc::SpeedControllerGroup m_right{m_rightfront, m_rightrear};
    frc::DifferentialDrive m_robotDrive{m_left, m_right};

    // conveyers
    WPI_TalonSRX m_vert_conveyer{10}; 
    WPI_TalonSRX m_hor_conveyer{11};

	// joysticks
	Joystick *m_stick, *m_stick_copilot;

	// gyro
	AHRS *ahrs;

    frc2::PIDController *m_pidController;
    frc::Timer m_timer;

    double rotateToAngleRate;           // Current rotation rate
    double speed_factor = 0.5;


	double TrimSpeed (double s, double max) {
	double result = s > max ? max : s;
	result = result < -max ? -max : result;
	return result;
	}

	double ScaleSpeed (double s, double scale) {
	return s * scale;
	}

	double ConvertRadsToDegrees (double rads) {
	const static double conversion_factor = 180.0/3.141592653589793238463;
	return rads * conversion_factor;
	}

public:

	void RobotInit() {

		/*************** shooter setup ***********************/

		m_shooter_star->ConfigFactoryDefault();
		m_shooter_port->ConfigFactoryDefault();

		// one follower and one reversed
		m_shooter_port->Follow(*m_shooter_star);
    	m_shooter_star->SetInverted(true);
    	m_shooter_port->SetInverted(false);
		
		// breaking mode
		m_shooter_star->SetNeutralMode(NeutralMode::Coast);
		m_shooter_port->SetNeutralMode(NeutralMode::Coast);

        /* feedback sensor */
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

		/***************** joysticks *************************/

		m_stick = new Joystick(k_joystick_pilot);
		m_stick_copilot = new Joystick(k_joystick_copilot);

		/********************* Gyro ****************************/

		try {
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
		} catch (std::exception &ex){
			std::string what_string = ex.what();
			std::string err_msg("Error instantiating navX MXP:  " + what_string);
			const char *p_err_msg = err_msg.c_str();
			DriverStation::ReportError(p_err_msg);
		}

		/* this is used to tune the PID numbers with shuffleboard
		frc::SmartDashboard::PutNumber("kP", kP);
		frc::SmartDashboard::PutNumber("kI", kI);
		frc::SmartDashboard::PutNumber("kD", kD);
		frc::SmartDashboard::PutNumber("MaxRotateRate", MaxRotateRate);
		*/

		/********************** stuff ************************/

		rotateToAngleRate = 0.0f;
		m_robotDrive.SetExpiration(0.1);
	}

	void TeleopInit() {
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

	void TeleopPeriodic() {

		/**************** buttons ******************/

		bool reset_yaw_button_pressed = m_stick->GetRawButton(1);  // reset gyro angle
		bool auto_shoot_button = m_stick_copilot->GetRawButton(2);
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

		/******************** joysticks **************************/

		double field_rel_X = m_stick->GetRawAxis(2);
		double field_rel_Y = -m_stick->GetRawAxis(3);
		double conveyer_Y = m_stick_copilot->GetRawAxis(3);
		double shooter_X = m_stick_copilot->GetRawAxis(0);
		double shooter_Y = m_stick_copilot->GetRawAxis(1);
		double shooter_R = sqrt(shooter_X*shooter_X + shooter_Y*shooter_Y);

		frc::SmartDashboard::PutNumber("Angle", ahrs->GetAngle());
		frc::SmartDashboard::PutNumber("Shooter Magnitude", shooter_R);

		/****************** move stuff *******************/

		// reset gyro angle
		if ( reset_yaw_button_pressed ) {
			ahrs->ZeroYaw();
		}

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
		/* get gamepad axis */
		//double leftYstick = _joy->GetY();
		//double motorOutput = m_shooter_star->GetMotorOutputPercent();

		/* while button1 is held down, closed-loop on target velocity */
		if (auto_shoot_button) {
        	/* Speed mode */
			/* Convert 500 RPM to units / 100ms.
			 * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
			// MJS: Falcon sensor reports 2048 units/rev

			double targetVelocity_UnitsPer100ms = 3000.0 * 2048 / 600;
        	m_shooter_star->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms); 

        } else {
			/* Percent voltage mode */
			m_shooter_star->Set(ControlMode::PercentOutput, shooter_Y);
		}
	}

	void AutonomousInit() {
		m_timer.Reset();
		m_timer.Start();
	}

	void AutonomousPeriodic() {
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

};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

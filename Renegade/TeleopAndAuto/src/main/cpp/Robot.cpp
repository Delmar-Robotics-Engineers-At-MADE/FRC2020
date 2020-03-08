
#include <iostream>
#include <string>
#include <Thread>

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
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

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
	const static double kMinTargetAreaPercent = 0.1;
	const static double kConveyerSpeed = 0.5;
	const static double kIdleShooterSpeed = 8000;

	
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
	double shooter_speed_in_units = 0.0;
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

	// sensors
	AHRS *ahrs;
	frc::DigitalInput eye_collector{0}; // photo eye at collector
	frc::DigitalInput eye_turret{1}; // photo eye at turret
	frc::DigitalInput hall_effect{2}; // hall effect sensor on turret

	// pid and timer
    frc2::PIDController *m_pidController;
    frc::Timer m_timer;

	// limelight
	std::shared_ptr<NetworkTable> m_limetable;  // for LimeLight

	// misc members
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

		/***************** limelight *************************/

		m_limetable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
		//m_limetable->PutNumber("camMode",0.0); // camera in normal CV mode
		//m_limetable->PutNumber("ledMode",1.0); // LED off
		//m_limetable->PutNumber("stream",0.0);  // secondary camera side-by-side

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

		// for testing shooter ranges
		frc::SmartDashboard::PutNumber("kP", kP);

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
		// used to tune shooter trajectory
		frc::SmartDashboard::PutNumber("shoot speed", shooter_speed_in_units);
		m_pidController = new frc2::PIDController (kP, kI, kD);
		m_pidController->SetTolerance(8, 8);  // within 8 degrees of target is considered on set point

		// ramp up shooter slowly
		m_timer.Reset();
		m_timer.Start();
		while (m_timer.Get() < 3) {
			if (m_timer.Get() > 1) {
				m_shooter_star->Set(ControlMode::Velocity, -2*kIdleShooterSpeed/4);
			} else if (m_timer.Get() > 2) {
				m_shooter_star->Set(ControlMode::Velocity, -3*kIdleShooterSpeed/4);
			}
		} // while timer
		m_shooter_star->Set(ControlMode::Velocity, -kIdleShooterSpeed);
	}

	void TeleopPeriodic() {

		/**************** buttons ******************/

		bool reset_yaw_button_pressed = m_stick->GetRawButton(1);  // reset gyro angle
		bool auto_shoot_button =  m_stick_copilot->GetRawButton(2);
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

		//frc::SmartDashboard::PutNumber("Angle", ahrs->GetAngle());
		//frc::SmartDashboard::PutNumber("Shooter Magnitude", shooter_R);

		/******************* limelight ****************************/

		double targetSeen = m_limetable->GetNumber("tv",0.0);
		double targetArea = m_limetable->GetNumber("ta",0.0);

		if (targetSeen != 0.0) {
		  frc::SmartDashboard::PutNumber("Targ Area", targetArea);
		  if (targetArea > kMinTargetAreaPercent) {  // tv is true if there is a target detected
			//double targetOffsetAngle_Horizontal = m_limetable->GetNumber("tx",0.0);
			double targetOffsetAngle_Vertical = m_limetable->GetNumber("ty",0.0);   
			//double targetSkew = m_limetable->GetNumber("ts",0.0);
			double targetWidth = m_limetable->GetNumber("tlong",0.0);

			frc::SmartDashboard::PutNumber("Targ Width", targetWidth);
			frc::SmartDashboard::PutNumber("Targ Vert", targetOffsetAngle_Vertical);
		  }
		}


		/****************** move stuff *******************/

		// reset gyro angle
		if ( reset_yaw_button_pressed ) {
			ahrs->ZeroYaw();
		}

		// shooter
		// if (shooter_R > 0.1) {
		// double targetVelocity_UnitsPer100ms = -1 * shooter_R * 1500.0 * 2048 / 600;
		// 	m_shooter_star->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms);
		// //frc::SmartDashboard::PutNumber("shooter target", targetVelocity_UnitsPer100ms);
		// } else {
		// 	m_shooter_star->Set(ControlMode::Velocity, 0.0);
		// }

		// power conveyer, which does not have encoders
		//frc::SmartDashboard::PutNumber("conveyer power", conveyer_Y);
		

		if (abs(field_rel_X) > kGamepadDeadZone || abs(m_stick->GetRawAxis(3)) > kGamepadDeadZone) {
			rotateToAngle = true;
			double angle =  90 - ConvertRadsToDegrees(atan(field_rel_Y/abs(field_rel_X)));
			angle = copysign(angle, field_rel_X); // make angle negative if X is negative
			m_pidController->SetSetpoint(angle);
		}
		//frc::SmartDashboard::PutNumber ("Angle set point", m_pidController->GetSetpoint());
		//frc::SmartDashboard::PutNumber ("X", field_rel_X);
		//frc::SmartDashboard::PutNumber ("Y", field_rel_Y);

		rotateToAngleRate = m_pidController->Calculate(ahrs->GetAngle());
		// trim the speed so it's not too fast
		rotateToAngleRate = TrimSpeed(rotateToAngleRate, kMaxRotateRate);

		// high and low speed scaling
		bool slow_gear_button_pressed = m_stick->GetRawButton(5);
		bool high_gear_button_presssed = m_stick->GetRawButton(7);
		if (slow_gear_button_pressed) {speed_factor = kSlowSpeedFactor;}
		else if (high_gear_button_presssed) {speed_factor = kFastSpeedFactor;}
		//frc::SmartDashboard::PutNumber ("Drive Speed Factor", speed_factor);

		try {

			if (rotateToAngle) {
			// MJS: since it's diff drive instead of mecanum drive, use tank method for rotation
			//frc::SmartDashboard::PutNumber("rotateToAngleRate", rotateToAngleRate);
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

		// ++++++++++++++++++++ shooter +++++++++++++++++++++++++++

		// double targetVel_UnitsPer100ms = -1 * 3000.0 * 2048 / 600;
		// frc::SmartDashboard::PutNumber ("Shooter speed", targetVel_UnitsPer100ms);
		// if (shooter_R > 0.1) {
		// 	targetVel_UnitsPer100ms = -1 * shooter_R * 1500.0 * 2048 / 600;
		// 	// //frc::SmartDashboard::PutNumber("shooter target", targetVelocity_UnitsPer100ms);
		// } else if (auto_shoot_button) {
		// 	/* while button1 is held down, closed-loop on target velocity */
		// 	// MJS: Falcon sensor reports 2048 units/rev
		// 	//targetVelocity_UnitsPer100ms = frc::SmartDashboard::GetNumber ("Shooter speed", 0.0);
		// 	// targetVel_UnitsPer100ms = -1 * 3000.0 * 2048 / 600;
		// 	m_shooter_star->Set(ControlMode::PercentOutput, 0.0);
        // } else {
		// 	targetVel_UnitsPer100ms = -1 * 1000.0 * 2048 / 600;
		// }

		double conveyer_speed = kConveyerSpeed * conveyer_Y;
		if (auto_shoot_button) {
			m_limetable->PutNumber("ledMode",3.0); // LED on
			shooter_speed_in_units = frc::SmartDashboard::GetNumber("shoot speed", 0.0);
			m_shooter_star->Set(ControlMode::Velocity, -shooter_speed_in_units);
			frc::SmartDashboard::PutNumber("shooter err", m_shooter_star->GetClosedLoopError());
			//if (m_shooter_star->GetClosedLoopError < kShooterSpeedTolerance)
		} else {
			m_limetable->PutNumber("ledMode",1.0); // LED off
			shooter_speed_in_units = kIdleShooterSpeed;
			m_shooter_star->Set(ControlMode::Velocity, -shooter_speed_in_units);
		}
		m_vert_conveyer.Set(conveyer_speed);
		

		// test digital sensors
		bool eye_value = eye_collector.Get();
		bool hall_value = hall_effect.Get();
		//frc::SmartDashboard::PutNumber("Eye", eye_value);
		frc::SmartDashboard::PutNumber("Hall Effect", hall_value);
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

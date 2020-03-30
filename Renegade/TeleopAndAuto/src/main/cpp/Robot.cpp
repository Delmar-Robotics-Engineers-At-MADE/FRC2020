
#include <iostream>
#include <string>
#include <Thread>

#include <frc/TimedRobot.h>
#include <frc/DigitalInput.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/DoubleSolenoid.h>
#include <frc/util/color.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>


#include "ctre/Phoenix.h"
#include "AHRS.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include "Constants.h"

#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

using namespace frc;

    // constants

    const static int k_joystick_pilot = 0;
    const static int k_joystick_copilot = 1;
	
	const static double kToleranceDegrees = 2.0f;
	const static double kMaxRotateRate = 1.0;
	const static double kGamepadDeadZone = 0.15;
	const static double kSlowSpeedFactor = 0.7;
	const static double kFastSpeedFactor = 0.9;
	const static double kMinTargetAreaPercent = 0.1;
	const static double kDriveAgainstCPSpeed = 0.25;

	const static double kConveyerSpeed = 0.75;
	const static double kIntakeSpeed = 0.7;
	const static double kIntakeDelayArrival = 1;
	const static double kIntakeDelayGap = 0.05;

	const static double kIdleShooterSpeed = 6000;
	const static double kMaxShooterSpeedError = 3500;  // move conveyer automatically when speed is good
	const static double kInitialShooterSlope = 300;

	const static double kMinColorConfidence = 0.85;
	const static double kControlPanelSpeed = 0.8;

	const static double kTurretSpeedInitial = 0.4;
	const static double kTurretSpeedMax = 0.6;
	const static long kMaxTurretInitialSeek = -4000; // in encoder counts
	const static long kTurretTolerance = 500; // in encoder counts
	const static long kTurretLimitPort = 15000;
	const static long kTurretLimitStarboard = -15000;
	const static long kTurretUP = 1;  // basicaly zero, but needs to be non-zero
	const static long kTurretRIGHT = 7000;
	const static long kTurretDOWN = 10000;
	const static long kTurretLEFT = -7000;
	const static long kLimelightTolerance = 5; // degrees

	const static long kPixyTolerance = 5; // X values
	const static double kChaseBallSpeed = 0.2;

	const static double kAutoDriveSpeed = 0.5;
	const static long kAutoDriveDistance = 6000;


	/* stock color set */
	static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
	static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
	static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
	static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
	static constexpr frc::Color kNoColor = frc::Color(0,0,0);
	

	/* our mockup control panel
	static constexpr frc::Color kBlueTarget = frc::Color(0.175, 0.436, 0.388);
	static constexpr frc::Color kGreenTarget = frc::Color(0.206, 0.545, 0.248);
	static constexpr frc::Color kRedTarget = frc::Color(0.424, 0.386, 0.190);
	static constexpr frc::Color kYellowTarget = frc::Color(0.330, 0.525, 0.145);
	static constexpr frc::Color kNoColor = frc::Color(0,0,0);
	*/
	
	const static double kPtunedGyro = 0.05;
	const static double kItunedGyro = 0.0;
	const static double kDtunedGyro = 0.0;
	const static double kPtunedPixy = 0.01;
	const static double kItunedPixy = 0.0;
	const static double kDtunedPixy = 0.0;
	const static double kPturret = 0.01;
	const static double kIturret = 0.003;
	const static double kDturret = 0.0;

class Robot: public TimedRobot {

	/*
	TalonFX 0	Left Shooter Motor
	TalonSRX 1	Left Froward Drive Motor
	TalonSRX 2	Left Aft Drive Motor
	TalonSRX 3	Intake Motor
	TalonSRX 5	Shooter Turret Rotator
	TalonSRX 7	Control Panel Rotator
	TalonSRX 10	Vertical Conveyor
	TalonSRX 11	Horizontal Conveyor
	TalonSRX 13	Right Forward Drive Motor
	TalonSRX 14	Right Aft Drive Motor
	TalonFX 15	Right Shooter Motor
	*/

	// shooter 
	TalonFX * m_shooter_star = new TalonFX(15); // 15 is starboard, 0 is port
	TalonFX * m_shooter_port = new TalonFX(0); // 15 is starboard, 0 is port
	double m_shooter_slope = 0.0;
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

    // conveyers and intake
    WPI_TalonSRX m_vert_conveyer{10}; 
    WPI_TalonSRX m_hor_conveyer{11};
    WPI_TalonSRX m_intake{3};

	// turret and control panel gizmo
	TalonSRX * m_turret = new TalonSRX(5); 
	WPI_TalonSRX m_control_spinner{7}; 

	// solenoids
	frc::DoubleSolenoid m_ponytail_solenoid{0, 1};

	// joysticks
	Joystick *m_stick, *m_stick_copilot;

	// sensors
	AHRS *ahrs;
	frc::DigitalInput eye_intake{0}; // photo eye at collector
	frc::DigitalInput eye_turret{1}; // photo eye at turret
	frc::DigitalInput hall_effect{2}; // hall effect sensor on turret
	static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
	rev::ColorSensorV3 m_colorSensor{i2cPort};
	rev::ColorMatch m_colorMatcher;

	// state machine for counting rotations of control panel
	enum RotateStates {
		kUnknownState = 0,
		kOffStartingColor,
		kOnStartingColor,
		kCompletedRotations
	};
	RotateStates m_wheel_state = kUnknownState;
	frc::Color m_starting_color = kNoColor;
	int m_half_rotation_count = 0;
	// state machine for spinning to color
	enum RotateToColorStates {
		kOffTargetColor = 0,
		kOnTargetColor,
		kToColorComplete
	};
	RotateToColorStates m_wheel_state_to_color = kOffTargetColor;
	bool m_need_to_reset_spinner = false;

	// for moving turret
	bool m_need_to_reset_manual_turret_move = false;

	// state machine for intake
	enum IntakeStates {
		kBallJustArrived = 0, 
		kBallInBreach,
		kBallJustLeft,
		kBreachEmpty
	};
	IntakeStates m_intake_state = kBreachEmpty;

	// pid and timer
    frc2::PIDController *m_pidController_gyro; // for orienting robot with gyro
	frc2::PIDController *m_pidController_limelight_robot;
	frc2::PIDController *m_pidController_limelight_turret;
	frc2::PIDController *m_pidController_pixycam;
    frc::Timer m_timer;

	// limelight
	std::shared_ptr<NetworkTable> m_limetable;  // for LimeLight
	std::shared_ptr<NetworkTable> m_pixytable;  // for Pixy Cam

	// misc members
    double rotateToAngleRate;           // Current rotation rate
    double speed_factor = 0.5;
	frc::SendableChooser<std::string> m_chooser;
	frc::SendableChooser<std::string> m_chooser_options_dir;
	frc::SendableChooser<std::string> m_chooser_options_speed;
	frc::SendableChooser<std::string> m_chooser_options_wait;
	const std::string kAutoNameTestWheels = "Move Wheels";
	const std::string kAutoNameMoveOnly = "Move Only";
	const std::string kAutoNameJustInit = "Just Initialize";
	const std::string kAutoNameShootAndMove = "Shoot/Move";
	const std::string kAutoOptionForward = "Forward";
	const std::string kAutoOptionBackward = "Backard";
	const std::string kAutoOptionNoMove = "No Move";
	const std::string kAutoOptionFast = "Fast";
	const std::string kAutoOptionSlow = "Slow";
	const std::string kAutoOptionWait = "Wait";
	const std::string kAutoOptionNoWait = "No wait";
	std::string m_autoSelected;
	std::string m_autoSelected_options_dir;
	std::string m_autoSelected_options_speed;
	std::string m_autoSelected_options_wait;
	bool m_do_once_inited = false;

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

		/*************** drive motor setup ***********************/

		m_leftfront.ConfigFactoryDefault();
		m_leftrear.ConfigFactoryDefault();
		m_rightfront.ConfigFactoryDefault();
		m_rightrear.ConfigFactoryDefault();

		// breaking mode
		m_leftfront.SetNeutralMode(NeutralMode::Brake);
		m_leftrear.SetNeutralMode(NeutralMode::Brake);
		m_rightfront.SetNeutralMode(NeutralMode::Brake);
		m_rightrear.SetNeutralMode(NeutralMode::Brake);

        /* feedback sensor */
		m_leftfront.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		m_rightfront.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		m_leftfront.SetSensorPhase(false);
		m_rightfront.SetSensorPhase(false);

		/* set the peak and nominal outputs */
		m_leftfront.ConfigNominalOutputForward(0, kTimeoutMs);
		m_leftfront.ConfigNominalOutputReverse(0, kTimeoutMs);
		m_leftfront.ConfigPeakOutputForward(1, kTimeoutMs);
		m_leftfront.ConfigPeakOutputReverse(-1, kTimeoutMs);
		m_rightfront.ConfigNominalOutputForward(0, kTimeoutMs);
		m_rightfront.ConfigNominalOutputReverse(0, kTimeoutMs);
		m_rightfront.ConfigPeakOutputForward(1, kTimeoutMs);
		m_rightfront.ConfigPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 */
		m_leftfront.Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_leftfront.Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		m_leftfront.Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_leftfront.Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_rightfront.Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_rightfront.Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		m_rightfront.Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_rightfront.Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

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

		/************** turret setup **********************/

		m_turret->ConfigFactoryDefault();
		// int absolutePosition = m_turret->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		// m_turret->SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);

		/* choose the sensor and sensor direction */
		m_turret->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, kPIDLoopIdx,kTimeoutMs);
		m_turret->SetSensorPhase(false);

		/* set the peak and nominal outputs, 12V means full */
		m_turret->ConfigNominalOutputForward(0, kTimeoutMs);
		m_turret->ConfigNominalOutputReverse(0, kTimeoutMs);
		m_turret->ConfigPeakOutputForward(kTurretSpeedMax, kTimeoutMs);
		m_turret->ConfigPeakOutputReverse(-kTurretSpeedMax, kTimeoutMs);

		/* set closed loop gains in slot0 */
		m_turret->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_turret->Config_kP(kPIDLoopIdx, 0.2, kTimeoutMs);
		m_turret->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_turret->Config_kD(kPIDLoopIdx, 0.5, kTimeoutMs);

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

		/***************** pixycam *************************/

		m_pixytable = nt::NetworkTableInstance::GetDefault().GetTable("PixyBlocks");

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

		/* this is used to tune the PID numbers with shuffleboard */
		frc::SmartDashboard::PutNumber("kP", kPtunedPixy);
		frc::SmartDashboard::PutNumber("kI", kItunedPixy);
		frc::SmartDashboard::PutNumber("kD", kDtunedPixy);
		
		frc::SmartDashboard::PutNumber("shoot slope", kInitialShooterSlope);
		
		// control panel colors
		m_colorMatcher.AddColorMatch(kBlueTarget);
		m_colorMatcher.AddColorMatch(kGreenTarget);
		m_colorMatcher.AddColorMatch(kRedTarget);
		m_colorMatcher.AddColorMatch(kYellowTarget);

		/********************** stuff ************************/

		rotateToAngleRate = 0.0f;
		m_robotDrive.SetExpiration(0.1);

		m_chooser.SetDefaultOption(kAutoNameShootAndMove, kAutoNameShootAndMove);
		m_chooser.AddOption(kAutoNameMoveOnly, kAutoNameMoveOnly);
		m_chooser.AddOption(kAutoNameJustInit, kAutoNameJustInit);
		m_chooser.AddOption(kAutoNameTestWheels, kAutoNameTestWheels);
		m_chooser_options_dir.SetDefaultOption(kAutoOptionForward,kAutoOptionForward);
		m_chooser_options_dir.AddOption(kAutoOptionBackward, kAutoOptionBackward);
		m_chooser_options_dir.AddOption(kAutoOptionNoMove, kAutoOptionNoMove);
		m_chooser_options_speed.SetDefaultOption(kAutoOptionFast,kAutoOptionFast);
		m_chooser_options_speed.AddOption(kAutoOptionSlow, kAutoOptionSlow);
		m_chooser_options_wait.AddOption(kAutoOptionWait, kAutoOptionWait);
		m_chooser_options_wait.AddOption(kAutoOptionNoWait, kAutoOptionNoWait);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		frc::SmartDashboard::PutData("Auto Move?", &m_chooser_options_dir);
		frc::SmartDashboard::PutData("Auto Fast?", &m_chooser_options_speed);
		frc::SmartDashboard::PutData("Auto Wait?", &m_chooser_options_wait);
	}

	void MoveTurretToManualPosition (long target_position) {
		m_turret->Set(ControlMode::Position, target_position);
		if (abs(target_position - m_turret->GetSelectedSensorPosition(0)) < kTurretTolerance) {
			m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
		}
	}
	
	void MoveTurretToStartingPosition() {

		bool turret_on_hall = false;
		// frc::SmartDashboard::PutString("turr state", "initial move");

		// code from WPI to set starting position
		// int absolutePosition = m_turret->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		// m_turret->SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
		m_turret->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

		
		// move turret to starting position using Hall sensor

		//int absolutePosition = m_turret->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		//m_turret->SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
		

		m_turret->Set(ControlMode::Position, kMaxTurretInitialSeek);  // positive moves turret clockwise

		m_timer.Reset(); m_timer.Start(); while (m_timer.Get() < 5) {
			turret_on_hall = !hall_effect.Get();
			if (turret_on_hall) { 
				m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
				// frc::SmartDashboard::PutString("turr state", "found hall");
				break; // exit loop
			}
			Wait(0.1);
		} // while
		m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
		// frc::SmartDashboard::PutString("turr state", "while done");

		turret_on_hall = !hall_effect.Get();
		if (!turret_on_hall) {
			// we didn't land on Hall sensor, go back the other way looking for it
			m_turret->Set(ControlMode::Position, -2 * kMaxTurretInitialSeek);
			m_timer.Reset(); m_timer.Start(); while (m_timer.Get() < 5) {
				turret_on_hall = !hall_effect.Get();
				if (turret_on_hall) { 
					m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
					// frc::SmartDashboard::PutString("turr state", "found hall");
					break; // exit loop
				}
				Wait(0.1);
			} // while		
		}
		m_turret->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);  // set zero position now
	}

	void TrackTargetWithRobot(double targetOffsetAngle) {
		m_pidController_limelight_robot->SetSetpoint(0);
		double diff_speed = m_pidController_limelight_robot->Calculate(targetOffsetAngle);

		// move robot
		m_robotDrive.TankDrive(-diff_speed, diff_speed, false);

		// check to see if we're within tolerance, and if so, reset
		if (m_pidController_limelight_robot->AtSetpoint()) {
			m_pidController_limelight_robot->Reset(); // clears out integral state, etc
		}
	}

	void ChasePowerCells() {
		bool ball_seen = m_pixytable->GetNumber("STATUS", -1);
		if (ball_seen == 1) {
			double pixy_X = 167 - m_pixytable->GetNumber("X",0.0);
			frc::SmartDashboard::PutString("Balls", "yes");
			frc::SmartDashboard::PutNumber("Ball X", pixy_X);

			double diff_speed = m_pidController_pixycam->Calculate(pixy_X);

			// move robot
			frc::SmartDashboard::PutNumber("Ball chase", diff_speed);
			m_robotDrive.TankDrive(-diff_speed+kChaseBallSpeed, diff_speed+kChaseBallSpeed, false);

			// don't reset pid controller
		} else if (ball_seen == 0) { // no balls seen
			frc::SmartDashboard::PutString("Balls", "no");
		} else { // error of some kind
			frc::SmartDashboard::PutString("Balls", "error");
		}
	}

	bool TrackTargetWithTurret(double targetOffsetAngle) {
		bool result = false;  // return true when on set point
		targetOffsetAngle = -targetOffsetAngle; 
		double current_pos = m_turret->GetSelectedSensorPosition(0);
		if (current_pos > kTurretLimitStarboard && current_pos < kTurretLimitPort) {
			// frc::SmartDashboard::PutString("turr tracking", "safe");
			m_pidController_limelight_turret->SetSetpoint(0);  // try moving this to RobotInit() and doing it only once
			double new_speed = m_pidController_limelight_turret->Calculate(targetOffsetAngle);
			// frc::SmartDashboard::PutNumber("turr speed", new_speed);
			// frc::SmartDashboard::PutNumber("off ang", targetOffsetAngle);

			// move turret
			m_turret->Set(ControlMode::PercentOutput, new_speed);

			result = m_pidController_limelight_turret->AtSetpoint();

			// hold position; don't reset when on target
		} else {
			// frc::SmartDashboard::PutString("turr tracking", "unsafe");
			m_turret->Set(ControlMode::PercentOutput, 0);
		}
		return result;
	}

	void TeleopInit() {
		DoOnceInit();
		RepeatableInit();
		m_wheel_state = kUnknownState;
		m_starting_color = kNoColor;
		m_half_rotation_count = 0;
		m_intake_state = kBreachEmpty;
	}

	void DoOnceInit() {
		if (!m_do_once_inited) {

			ahrs->ZeroYaw();

			double P_gyro = kPtunedGyro;
			double I_gyro = kItunedGyro;
			double D_gyro = kDtunedGyro;
			double P_pixy = kPtunedPixy;  // try using the same pid values for rotating to balls
			double I_pixy = kItunedPixy;
			double D_pixy = kDtunedPixy;

			/* used to tune PID numbers */
			double P = frc::SmartDashboard::GetNumber("kP", P_pixy);
			double I = frc::SmartDashboard::GetNumber("kI", I_pixy);
			double D = frc::SmartDashboard::GetNumber("kD", D_pixy);
					
			
			// PIDs
			m_pidController_gyro = new frc2::PIDController (P_gyro, I_gyro, D_gyro);
			m_pidController_gyro->SetTolerance(8, 8);  // within 8 degrees of direction is considered on set point
			m_pidController_limelight_robot = new frc2::PIDController (P_gyro, I_gyro, D_gyro);
			m_pidController_limelight_robot->SetTolerance(kLimelightTolerance, kLimelightTolerance);  // within 8 degrees of target is considered on set point
			m_pidController_limelight_turret = new frc2::PIDController (kPturret, kIturret, kDturret);
			m_pidController_limelight_turret->SetTolerance(kLimelightTolerance, kLimelightTolerance);  // within 8 degrees of target is considered on set point
			m_pidController_pixycam = new frc2::PIDController (P, I, D);
			m_pidController_pixycam->SetTolerance(kPixyTolerance, kPixyTolerance);  // within 8 degrees of target is considered on set point
			m_pidController_pixycam->SetSetpoint(0);  // always use same setpoint

			// position turret
			MoveTurretToStartingPosition();
		}

		m_do_once_inited = true;
	}

	void RepeatableInit() {

		// bring up shooter
		m_shooter_star->Set(ControlMode::Velocity, -kIdleShooterSpeed);
		m_shooter_slope = frc::SmartDashboard::GetNumber("shoot slope", kInitialShooterSlope);

		// position ponytail up
		m_ponytail_solenoid.Set(frc::DoubleSolenoid::kForward);
	}

	std::string ColorToString (frc::Color color) {
		std::string result;
		if (color == kBlueTarget) {
			result = "Blue";
		} else if (color == kRedTarget) {
			result = "Red";
		} else if (color == kGreenTarget) {
			result = "Green";
		} else if (color == kYellowTarget) {
			result = "Yellow";
		} else {
			result = "Unknown";
		}	
		return result;
	}

	void ResetSpinner() {
		m_robotDrive.TankDrive(0, 0); // stop pressing into CP
		m_wheel_state = kUnknownState;
		m_half_rotation_count = 0;
		m_wheel_state_to_color = kOffTargetColor;
		m_control_spinner.Set(0.0);
		m_ponytail_solenoid.Set(frc::DoubleSolenoid::kForward); // raise spinner
	}

	void SpinThreeTimes() {
		std::string colorString;
		double confidence = 0.0;
		frc::SmartDashboard::PutNumber("CP COMPLETE", false);

		m_robotDrive.TankDrive(-kDriveAgainstCPSpeed, -kDriveAgainstCPSpeed); // press against control panel
		MoveTurretToManualPosition(kTurretUP); // hold turret in position
		m_ponytail_solenoid.Set(frc::DoubleSolenoid::kReverse); // lower spinner here
		
		frc::Color detectedColor = m_colorSensor.GetColor();
		frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

		switch (m_wheel_state) {
		case kUnknownState: 
			m_control_spinner.Set(kControlPanelSpeed); // spinning
			if (confidence > kMinColorConfidence) {
				m_wheel_state = kOnStartingColor;
				m_starting_color = matchedColor;
			}
			break;
		case kOnStartingColor:
			if (confidence > kMinColorConfidence && !(matchedColor == m_starting_color)) {
				m_wheel_state = kOffStartingColor;
			}
			if (m_half_rotation_count >= 7) { // 3.5 times around
				m_wheel_state = kCompletedRotations;
			}
			break;
		case kOffStartingColor:
			if (confidence > kMinColorConfidence && matchedColor == m_starting_color) {
				m_wheel_state = kOnStartingColor;
				m_half_rotation_count++;
			}
			break;
		case kCompletedRotations:
			m_control_spinner.Set(0.0); // stop
			//m_ponytail_solenoid.Set(frc::DoubleSolenoid::kForward); // raise spinner
			frc::SmartDashboard::PutNumber("CP COMPLETE", true);
			break;
		}
		frc::SmartDashboard::PutString("color", ColorToString(matchedColor));
		frc::SmartDashboard::PutNumber("confidence", confidence);
		
	}

	void SpinToColor(frc::Color spin_to_color) {
		std::string colorString;
		double confidence = 0.0;
		frc::SmartDashboard::PutNumber("CP COMPLETE", false);

		m_robotDrive.TankDrive(-kDriveAgainstCPSpeed, -kDriveAgainstCPSpeed); // press against control panel
		MoveTurretToManualPosition(kTurretUP); // hold turret in position
		m_ponytail_solenoid.Set(frc::DoubleSolenoid::kReverse); // lower spinner here

		frc::Color detectedColor = m_colorSensor.GetColor();
		frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

		// spin to color
		switch (m_wheel_state_to_color) {
			case kOnTargetColor:
				// stop; we're there
				m_wheel_state_to_color = kToColorComplete;
				break;
			case kOffTargetColor:
				if (confidence > kMinColorConfidence && matchedColor == spin_to_color) {
					m_wheel_state_to_color = kOnTargetColor;
				} else {
					m_control_spinner.Set(kControlPanelSpeed);
				}
				break;
			case kToColorComplete:
				m_control_spinner.Set(0.0);
				//m_ponytail_solenoid.Set(frc::DoubleSolenoid::kForward); // raise spinner here
				frc::SmartDashboard::PutNumber("CP COMPLETE", true);
				break;
		}
		frc::SmartDashboard::PutString("color", ColorToString(matchedColor));
		frc::SmartDashboard::PutNumber("confidence", confidence);
	}

	void AutoIntakeBalls() {
		if (eye_turret.Get()) { // there's a ball under the turret; no more collecting until that's gone
			m_intake.Set(0.0);
			m_vert_conveyer.Set(0.0);
		} else { // turret on-deck empty; ok to intake
			m_vert_conveyer.Set(-kConveyerSpeed);
			// frc::SmartDashboard::PutNumber("intake state", m_intake_state);
			switch (m_intake_state) {
				case kBallJustArrived:
					m_timer.Reset();
					m_timer.Start();
					m_intake_state = kBallInBreach;
					break;
				case kBallInBreach: // run long enough to get ball into conveyer, then stop intake
					if (m_timer.Get() < kIntakeDelayArrival) {
						m_intake.Set(-kIntakeSpeed);
					} else {
						m_intake_state = kBallJustLeft;
						m_intake.Set(-0.0);
					}
					break;
				case kBallJustLeft:  //  keep conveyer stopped for a period to create space
					if (m_timer.Get() < kIntakeDelayArrival + kIntakeDelayGap) {
						m_intake.Set(-0.0);
					} else { // ok to run conveyer now
						m_intake_state = kBreachEmpty;
					}
					break;
				case kBreachEmpty:
					if (eye_intake.Get()) { // a ball just arrived at breach
						m_intake_state = kBallJustArrived;
					} else { // no balls yet
						m_intake.Set(-kIntakeSpeed);
						m_vert_conveyer.Set(-kConveyerSpeed);
					}
			} // switch
		} // if turret is not on-deck
	}

	void OperateConveyer (bool conveyer_in_button, bool conveyer_out_button, double &conveyer_speed) {
			// frc::SmartDashboard::PutNumber("conv man ok", true);
			if (conveyer_in_button) {
				conveyer_speed = -kConveyerSpeed;
			} else if (conveyer_out_button) {
				conveyer_speed = kConveyerSpeed;
			}
			m_vert_conveyer.Set(conveyer_speed);
	}

	void OperateShooter(bool &manual_conveyer_ok, double &conveyer_speed,
		bool manual_boost, bool manual_deboost) {

		double targetSeen = m_limetable->GetNumber("tv",0.0);
		double targetArea = m_limetable->GetNumber("ta",0.0);

		double targetOffsetAngle_Vertical = 0.0;
		double targetOffsetAngle_Horizontal = 0.0;
		if (targetSeen != 0.0) {
		  if (targetArea > kMinTargetAreaPercent) {  // tv is true if there is a target detected
			targetOffsetAngle_Horizontal = m_limetable->GetNumber("tx",0.0);
			targetOffsetAngle_Vertical = m_limetable->GetNumber("ty",0.0);   
		  }
		}
		conveyer_speed = 0.0; 
		manual_conveyer_ok = false;

		double shooter_speed_in_units = kIdleShooterSpeed;
		if (targetSeen != 0.0) {
			// frc::SmartDashboard::PutString("turr mode", "tracking");
			bool limelight_on_target = TrackTargetWithTurret(targetOffsetAngle_Horizontal);

			if (targetOffsetAngle_Vertical < -18) {
				shooter_speed_in_units = 23000;  // max out shooter if far away
			} else {
				shooter_speed_in_units = 12696.1 - m_shooter_slope * targetOffsetAngle_Vertical; // originally 317.502
			}
			if (manual_boost) {shooter_speed_in_units *= 1.1;}
			else if (manual_deboost) {shooter_speed_in_units *= 0.9;}
			// frc::SmartDashboard::PutNumber("shoot speed", shooter_speed_in_units);
			double shooter_speed_error = m_shooter_star->GetClosedLoopError();
			frc::SmartDashboard::PutNumber("flywheel err", shooter_speed_error);
			if (shooter_speed_error < kMaxShooterSpeedError && limelight_on_target) {
				// auto feed balls into shooter
				conveyer_speed = -kConveyerSpeed;
			} else {
				manual_conveyer_ok = true;
			}
		} else { // no target; permit manual control of conveyer
			manual_conveyer_ok = true;
			m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret; needs to hold position
			// frc::SmartDashboard::PutString("turr mode", "stopped");
		}
		m_shooter_star->Set(ControlMode::Velocity, -shooter_speed_in_units);

	}
	void TeleopPeriodic() {

		//double targetPositionRotations =  -2000 * shooter_Y; // positive moves turret clockwise
		//m_turret->Set(ControlMode::Position, targetPositionRotations); 

		// frc::SmartDashboard::PutNumber("turret pos", m_turret->GetSelectedSensorPosition(0));

		/****************************************** buttons and joysticks **************************************/

		/************************************** pilot 
		Left Stick	Robot relative arcade	
		Right Stick	Field relative arcade	
		D Pad	Rotate to compass pts	
		Button 1	Reset Yaw / (color)	
		Button 2	(color)
		Button 3	Spin control panel / (color)	
		Button 4	Auto chase balls / (color)
		Button 5	Spin to color	
		Button 7	Turbo speed		
		*/
		double robot_rel_X = m_stick->GetRawAxis(0);  // robot relative
		double robot_rel_Y = -m_stick->GetRawAxis(1);
		double field_rel_X = m_stick->GetRawAxis(2);  // field relative
		double field_rel_Y = -m_stick->GetRawAxis(3);
		double field_rel_R = sqrt(field_rel_X*field_rel_X + field_rel_Y*field_rel_Y);
		bool rotateToAngle = false;
		if ( m_stick->GetPOV() == 0) {
			m_pidController_gyro->SetSetpoint(0.0f);
			rotateToAngle = true;
		} else if ( m_stick->GetPOV() == 90) {
			m_pidController_gyro->SetSetpoint(90.0f);
			rotateToAngle = true;
		} else if ( m_stick->GetPOV() == 180) {
			m_pidController_gyro->SetSetpoint(179.9f);
			rotateToAngle = true;
		} else if ( m_stick->GetPOV() == 270) {
			m_pidController_gyro->SetSetpoint(-90.0f);
			rotateToAngle = true;
		}
		bool reset_yaw_button_pressed = false;  // reset gyro angle
		bool chase_cells_button = false;
		bool spin_control_panel_button = false;
		bool spin_to_color_pressed = false;
		frc::Color spin_to_color = kNoColor;
		if (m_stick->GetRawButton(5)) { // spin to color
			if (m_stick->GetRawButton(1)) { // blue
				spin_to_color = kRedTarget; // blue
				spin_to_color_pressed = true;
			} else if (m_stick->GetRawButton(2)) { // green
				spin_to_color = kYellowTarget; 
				spin_to_color_pressed = true;
			}else if (m_stick->GetRawButton(3)) { // red
				spin_to_color = kBlueTarget; 
				spin_to_color_pressed = true;
			}else if (m_stick->GetRawButton(4)) { // yellow
				spin_to_color = kGreenTarget; 
				spin_to_color_pressed = true;
			}
			// frc::SmartDashboard::PutString ("Spin to", ColorToString(spin_to_color));
		} else { // color buttons have normal functions
			reset_yaw_button_pressed = m_stick->GetRawButton(1);  // reset gyro angle
			chase_cells_button = m_stick->GetRawButton(4);
			spin_control_panel_button = m_stick->GetRawButton(3);
		}
		
		bool high_gear_button_presssed = m_stick->GetRawButton(7);

		/****************************************** co-pilot
		Left Stick	deploy hanger and hang	
		Right Stick	Manual intake in/out	
		D Pad	Turret positioning	
		Button 1	+ left Y to deploy hanger
		Button 2	Automatic shoot	
		Button 3	+ left Y to hang
		Button 4	Automatic intake	
		Button 5	Boost shooter up	
		Button 6	Conveyer in/up	
		Button 7	De-boost shooter down	
		Button 8	Conveyer down/out	
		*/ 
		double hanger_Y = m_stick_copilot->GetRawAxis(1);
		// double shooter_R = sqrt(shooter_X*shooter_X + shooter_Y*shooter_Y); // could be used for manual shooter speed
		double intake_Y = m_stick_copilot->GetRawAxis(3);

		long turret_manual_position = 0;
		if ( m_stick_copilot->GetPOV() == 0) {
			turret_manual_position = kTurretUP;
		} else if ( m_stick_copilot->GetPOV() == 90) {
			turret_manual_position = kTurretRIGHT;
		} else if ( m_stick_copilot->GetPOV() == 180) {
			turret_manual_position = kTurretDOWN;
		} else if ( m_stick_copilot->GetPOV() == 270) {
			turret_manual_position = kTurretLEFT;
		}

		bool hanger_deploy_button = m_stick_copilot->GetRawButton(1);
		bool auto_shoot_button =  m_stick_copilot->GetRawButton(2);
		bool hang_button = m_stick_copilot->GetRawButton(3);
		bool auto_intake_button =  m_stick_copilot->GetRawButton(4);
		bool boost_shooter_up_button =  m_stick_copilot->GetRawButton(5);
		bool boost_shooter_down_button =  m_stick_copilot->GetRawButton(7);
		bool conveyer_in_button =  m_stick_copilot->GetRawButton(6);
		bool conveyer_out_button =  m_stick_copilot->GetRawButton(8);

		/****************************************** limelight *****************************************************/



		/********************************************** move stuff ********************************************/

		//m_turret->Set(ControlMode::PercentOutput, shooter_Y);  // temporary test
		
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
			m_pidController_gyro->SetSetpoint(angle);
		}
		//frc::SmartDashboard::PutNumber ("Angle set point", m_pidController_gyro->GetSetpoint());
		//frc::SmartDashboard::PutNumber ("X", field_rel_X);
		//frc::SmartDashboard::PutNumber ("Y", field_rel_Y);

		rotateToAngleRate = m_pidController_gyro->Calculate(ahrs->GetAngle());
		// trim the speed so it's not too fast
		rotateToAngleRate = TrimSpeed(rotateToAngleRate, kMaxRotateRate);


		// if (slow_gear_button_pressed) {speed_factor = kSlowSpeedFactor;}
		if (high_gear_button_presssed) {speed_factor = kFastSpeedFactor;}
		else {speed_factor = kSlowSpeedFactor;}
		// frc::SmartDashboard::PutNumber ("Rotate ratio", abs(kMaxRotateRate - abs(rotateToAngleRate)) / kMaxRotateRate);

		try {
			if (rotateToAngle) {
				// MJS: since it's diff drive instead of mecanum drive, use tank method for rotation
				//frc::SmartDashboard::PutNumber("rotateToAngleRate", rotateToAngleRate);
				double left_power = rotateToAngleRate;
				double right_power = -rotateToAngleRate;
				if (abs(kMaxRotateRate - abs(rotateToAngleRate)) / kMaxRotateRate > 0.7) { 
					// add forard driving to rotation, to get field relative driving
					double addition = field_rel_R * speed_factor;
					left_power += addition;
					right_power += addition;
				}
				m_robotDrive.TankDrive(left_power, right_power, false);
				// frc::SmartDashboard::PutNumber ("Left power", left_power);
				// frc::SmartDashboard::PutNumber ("Right power", right_power);
			} else if (spin_control_panel_button || spin_to_color_pressed) {
				// let cp program control wheels
			} else {
				// not rotating; drive by stick
				m_robotDrive.ArcadeDrive(ScaleSpeed(robot_rel_Y, speed_factor), ScaleSpeed(robot_rel_X, speed_factor));
				m_pidController_gyro->Reset(); // clears out integral state, etc
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
		bool manual_conveyer_ok = false;
		double conveyer_speed = 0.0;
		
		if (auto_shoot_button) {
			m_limetable->PutNumber("ledMode",3.0); // LED on
			OperateShooter(manual_conveyer_ok, conveyer_speed, 
			    boost_shooter_up_button, boost_shooter_down_button);
		} else {
			m_limetable->PutNumber("ledMode",1.0); // LED off
			m_shooter_star->Set(ControlMode::Velocity, -kIdleShooterSpeed);
		}
		OperateConveyer(conveyer_in_button, conveyer_out_button, conveyer_speed);

		if (auto_intake_button) {
			// bring balls in and index using photo eye
			AutoIntakeBalls();
		} else { // enable manual control of intake
			m_intake.Set(-intake_Y * kIntakeSpeed);
		}
		// operate control panel
		if (spin_control_panel_button) {
			m_need_to_reset_spinner = true;
			SpinThreeTimes();
		} else if (spin_to_color_pressed) {
			m_need_to_reset_spinner = true;
			SpinToColor(spin_to_color);
		} else if (m_need_to_reset_spinner) {
			ResetSpinner();
			m_need_to_reset_spinner = false;
		}
		// frc::SmartDashboard::PutNumber("wheel state", m_wheel_state);

		// auto-chase balls
		if (chase_cells_button) {
			ChasePowerCells();
		}
		
		if (turret_manual_position != 0) {
			m_need_to_reset_manual_turret_move = true;
			MoveTurretToManualPosition(turret_manual_position);
		} else if (m_need_to_reset_manual_turret_move) {
			m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
			m_need_to_reset_manual_turret_move = false;
		}
		// frc::SmartDashboard::PutNumber("turret pos3", m_turret->GetSelectedSensorPosition(0));
	}

	void AutonomousInit() {
		DoOnceInit();
		RepeatableInit();
		m_autoSelected = m_chooser.GetSelected();
		m_autoSelected_options_dir = m_chooser_options_dir.GetSelected();
		m_autoSelected_options_speed = m_chooser_options_speed.GetSelected();
		m_autoSelected_options_wait = m_chooser_options_wait.GetSelected();
		// m_autoSelected = SmartDashboard::GetString("Auto Selector",
		//     kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameShootAndMove) {
			m_limetable->PutNumber("ledMode",3.0);
		}
		m_leftfront.SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
		m_rightfront.SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
		m_timer.Reset();
		m_timer.Start();
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameTestWheels) {
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
		} else if (m_autoSelected == kAutoNameMoveOnly) {
				double motor_speed = 0.0;
				if (m_autoSelected_options_dir == kAutoOptionForward) {
					motor_speed = kAutoDriveSpeed;
				} else if (m_autoSelected_options_dir == kAutoOptionBackward) {
					motor_speed = -kAutoDriveSpeed;
				}				
				if (m_autoSelected_options_speed == kAutoOptionFast) {
					motor_speed *= 2;
				}
				if (abs(m_leftfront.GetSelectedSensorPosition(0)) < kAutoDriveDistance) {
					m_robotDrive.TankDrive(motor_speed, motor_speed);
					// frc::SmartDashboard::PutString("Auto Mode", "moving");
				} else {
					m_robotDrive.TankDrive(0, 0);
					// frc::SmartDashboard::PutString("Auto Mode", "done moving");
				}
		} else if (m_autoSelected == kAutoNameShootAndMove) {
			// frc::SmartDashboard::PutString("Auto Mode", "kAutoNameShootAndMove");
			if (m_autoSelected_options_wait == kAutoOptionWait && m_timer.Get() < 3) {
				// frc::SmartDashboard::PutString("Auto Mode", "waiting");
				// do nothing for x secs		
			} else if (m_timer.Get() < 6) {
				// frc::SmartDashboard::PutString("Auto Mode", "shooting");
				bool manual_conveyer_ok; double conveyer_speed;
				OperateShooter(manual_conveyer_ok, conveyer_speed, false, false);
				OperateConveyer (false, false, conveyer_speed);
			} else { // after 6 seconds
				m_shooter_star->Set(ControlMode::Velocity, -kIdleShooterSpeed);
				m_vert_conveyer.Set(0);
				double motor_speed = 0.0;
				if (m_autoSelected_options_dir == kAutoOptionForward) {
					motor_speed = kAutoDriveSpeed;
				} else if (m_autoSelected_options_dir == kAutoOptionBackward) {
					motor_speed = -kAutoDriveSpeed;
				}				
				if (m_autoSelected_options_speed == kAutoOptionFast) {
					motor_speed *= 2;
				}
				// frc::SmartDashboard::PutString("Auto Mode", "should move");
				if (abs(m_leftfront.GetSelectedSensorPosition(0)) < kAutoDriveDistance) {
					m_robotDrive.TankDrive(motor_speed, motor_speed);
					// frc::SmartDashboard::PutString("Auto Mode", "moving");
				} else {
					m_robotDrive.TankDrive(0, 0);
					// frc::SmartDashboard::PutString("Auto Mode", "done moving");
				}
				// frc::SmartDashboard::PutNumber("encoder", m_leftfront.GetSelectedSensorPosition(0));
			}
		} else {
			// only doing init
			// frc::SmartDashboard::PutString("Auto Mode", "nothing");
		}
	}

};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

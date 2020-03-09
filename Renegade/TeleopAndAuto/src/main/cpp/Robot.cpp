
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
	const static double kMaxRotateRate = 0.5;
	const static double kGamepadDeadZone = 0.15;
	const static double kSlowSpeedFactor = 0.6;
	const static double kFastSpeedFactor = 8.0;
	const static double kMinTargetAreaPercent = 0.1;
	const static double kConveyerSpeed = 0.5;
	const static double kIntakeSpeed = 0.5;
	const static double kIdleShooterSpeed = 6000;
	const static double kMaxShooterSpeedError = 2000;  // move conveyer automatically when speed is good
	const static double kMinColorConfidence = 0.85;
	const static double kControlPanelSpeed = 0.5;

	/* stock color set
	static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
	static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
	static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
	static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
	*/

	// our mockup control panel
	static constexpr frc::Color kBlueTarget = frc::Color(0.175, 0.436, 0.388);
	static constexpr frc::Color kGreenTarget = frc::Color(0.206, 0.545, 0.248);
	static constexpr frc::Color kRedTarget = frc::Color(0.424, 0.386, 0.190);
	static constexpr frc::Color kYellowTarget = frc::Color(0.330, 0.525, 0.145);
	static constexpr frc::Color kNoColor = frc::Color(0,0,0);
	
	double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
	const static double kPtuned = 0.006;
	const static double kItuned = 0.0015;
	const static double kDtuned = 0.001;

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
	frc::DigitalInput eye_collector{0}; // photo eye at collector
	frc::DigitalInput eye_turret{1}; // photo eye at turret
	frc::DigitalInput hall_effect{2}; // hall effect sensor on turret
	static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
	rev::ColorSensorV3 m_colorSensor{i2cPort};
	rev::ColorMatch m_colorMatcher;

	// state machine for counting rotations of control panel
	enum RotateStates {
		kUnknownState = 0,
		kOffStartingColor,
		kOnStartingColor
	};
	int m_wheel_state = kUnknownState;
	int m_wheel_state_to_color = kOffTargetColor;
	frc::Color m_starting_color = kNoColor;
	int m_half_rotation_count = 0;
	// state machine for spinning to color
	enum RotateToColorStates {
		kOffTargetColor = 0,
		kOnTargetColor
	};
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

		/************** turret setup **********************/

		m_turret->ConfigFactoryDefault();
		int absolutePosition = m_turret->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		m_turret->SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);

		/* choose the sensor and sensor direction */
		m_turret->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, kPIDLoopIdx,kTimeoutMs);
		m_shooter_star->SetInverted(true);  // want positive to be clockwise
		m_turret->SetSensorPhase(false);

		/* set the peak and nominal outputs, 12V means full */
		m_turret->ConfigNominalOutputForward(0, kTimeoutMs);
		m_turret->ConfigNominalOutputReverse(0, kTimeoutMs);
		m_turret->ConfigPeakOutputForward(1, kTimeoutMs);
		m_turret->ConfigPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 */
		m_turret->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_turret->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		m_turret->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		m_turret->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

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

		// control panel colors
		m_colorMatcher.AddColorMatch(kBlueTarget);
		m_colorMatcher.AddColorMatch(kGreenTarget);
		m_colorMatcher.AddColorMatch(kRedTarget);
		m_colorMatcher.AddColorMatch(kYellowTarget);

		/********************** stuff ************************/

		rotateToAngleRate = 0.0f;
		m_robotDrive.SetExpiration(0.1);
	}

	void TeleopInit() {
		ahrs->ZeroYaw();

		m_wheel_state = kUnknownState;
		m_starting_color = kNoColor;
		m_half_rotation_count = 0;
		
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

		// move turret to starting position using Hall sensor

		//int absolutePosition = m_turret->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		//m_turret->SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
		m_timer.Reset();
		m_timer.Start();
		//double targetPositionRotations =  -2000 * shooter_Y; // positive moves turret clockwise
		//m_turret->Set(ControlMode::Position, targetPositionRotations); 

		while (m_timer.Get() < 3) {
			if (!hall_effect.Get()) { // false means turret is on sensor
				m_turret->Set(ControlMode::PercentOutput, 0.0); // stop turret
			}
		}

		bool turret_on_hall = !hall_effect.Get();
		if (!turret_on_hall) {
			// we didn't land on Hall sensor, go back the other way looking for it
			//targetPositionRotations = -1500; // positive moves turret clockwise
			//m_turret->Set(ControlMode::Position, targetPositionRotations);
		}
		frc::SmartDashboard::PutNumber("On Hall Effect", turret_on_hall);
		frc::SmartDashboard::PutNumber("turret pos", m_turret->GetSelectedSensorPosition(0));


		// ramp up shooter slowly
		// m_timer.Reset();
		// m_timer.Start();
		// while (m_timer.Get() < 3) {
		// 	if (m_timer.Get() > 1) {
		// 		m_shooter_star->Set(ControlMode::Velocity, -2*kIdleShooterSpeed/4);
		// 	} else if (m_timer.Get() > 2) {
		// 		m_shooter_star->Set(ControlMode::Velocity, -3*kIdleShooterSpeed/4);
		// 	}
		// } // while timer
		m_shooter_star->Set(ControlMode::Velocity, -kIdleShooterSpeed);
	}

	void SpinThreeTimes() {
		std::string colorString;
		double confidence = 0.0;

		m_ponytail_solenoid.Set(frc::DoubleSolenoid::kForward); // lower spinner here
		
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
				m_control_spinner.Set(0.0); // stop
			}
			break;
		case kOffStartingColor:
			if (confidence > kMinColorConfidence && matchedColor == m_starting_color) {
				m_wheel_state = kOnStartingColor;
				m_half_rotation_count++;
			}
			break;
		}
	}

	void SpinToColor(frc::Color spin_to_color) {
		std::string colorString;
		double confidence = 0.0;

		m_ponytail_solenoid.Set(frc::DoubleSolenoid::kForward); // lower spinner here

		frc::Color detectedColor = m_colorSensor.GetColor();
		frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

		// spin to color
		switch (m_wheel_state_to_color) {
			case kOnTargetColor:
				// stop; we're there
				m_control_spinner.Set(0.0);
				break;
			case kOffTargetColor:
				if (confidence > kMinColorConfidence && matchedColor == spin_to_color) {
					m_wheel_state_to_color = kOnTargetColor;
				} else {
					m_control_spinner.Set(kControlPanelSpeed);
				}
				break;
		}
	}

	void TeleopPeriodic() {

		//double targetPositionRotations =  -2000 * shooter_Y; // positive moves turret clockwise
		//m_turret->Set(ControlMode::Position, targetPositionRotations); 

		frc::SmartDashboard::PutNumber("turret pos", m_turret->GetSelectedSensorPosition(0));

		/****************************************** buttons and joysticks **************************************/

		/************************************** pilot 
		Left Stick	Robot relative arcade	
		Right Stick	Field relative arcade	
		D Pad	Rotate to compass pts	
		Button 1	Reset Yaw / color	
		Button 2	Hang / color	
		Button 3	color	
		Button 4	Spin control panel / color	
		Button 5	Spin to color	
		Button 6	6+8 to deploy hanger	
		Button 7	Turbo speed	
		Button 8	Hang	
		*/
		double robot_rel_X = m_stick->GetRawAxis(0);  // robot relative
		double robot_rel_Y = -m_stick->GetRawAxis(1);
		double field_rel_X = m_stick->GetRawAxis(2);  // field relative
		double field_rel_Y = -m_stick->GetRawAxis(3);
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
		bool reset_yaw_button_pressed = m_stick->GetRawButton(1);  // reset gyro angle
		bool hang_button = m_stick->GetRawButton(2);
		bool spin_control_panel_button = m_stick->GetRawButton(4);
		bool spin_to_color_pressed = false;
		frc::Color spin_to_color = kNoColor;
		if (m_stick->GetRawButton(5)) { // spin to color
			if (m_stick->GetRawButton(1)) { // blue
				spin_to_color = kBlueTarget; // these will have to be shifted
				spin_to_color_pressed = true;
			} else if (m_stick->GetRawButton(2)) { // green
				spin_to_color = kBlueTarget; 
				spin_to_color_pressed = true;
			}if (m_stick->GetRawButton(3)) { // red
				spin_to_color = kBlueTarget; 
				spin_to_color_pressed = true;
			}if (m_stick->GetRawButton(4)) { // yellow
				spin_to_color = kBlueTarget; 
				spin_to_color_pressed = true;
			}
		}
		bool deploy_hanger_pressed = false;
		if (m_stick->GetRawButton(6) && m_stick->GetRawButton(8)) {
			deploy_hanger_pressed = true;
		}
		bool high_gear_button_presssed = m_stick->GetRawButton(7);
		bool hang_pressed = false;
		if (m_stick->GetRawButton(5) && m_stick->GetRawButton(7)) {
			hang_pressed = true;
		}

		/****************************************** co-pilot
		Left Stick	Manual turret dir/speed	
		Right Stick	Manual intake in/out	
		D Pad	Turret positioning	
		Button 1		
		Button 2	Automatic shoot	
		Button 3		
		Button 4	Automatic intake	
		Button 5	Boost shooter up	
		Button 6	Conveyer in/up	
		Button 7	De-boost shooter down	
		Button 8	Conveyer down/out	
		*/ 
		double shooter_X = m_stick_copilot->GetRawAxis(0);  // manual turret operation
		double shooter_Y = m_stick_copilot->GetRawAxis(1);
		double shooter_R = sqrt(shooter_X*shooter_X + shooter_Y*shooter_Y);
		double intake_X = m_stick_copilot->GetRawAxis(2); // manual turret operation
		double intake_Y = m_stick_copilot->GetRawAxis(3);
		bool auto_shoot_button =  m_stick_copilot->GetRawButton(2);
		bool auto_intake_button =  m_stick_copilot->GetRawButton(4);
		bool boost_shooter_up_button =  m_stick_copilot->GetRawButton(5);
		bool boost_shooter_down_button =  m_stick_copilot->GetRawButton(7);
		bool conveyer_in_button =  m_stick_copilot->GetRawButton(6);
		bool conveyer_out_button =  m_stick_copilot->GetRawButton(8);


		//frc::SmartDashboard::PutNumber("Angle", ahrs->GetAngle());
		//frc::SmartDashboard::PutNumber("Shooter Magnitude", shooter_R);


		/****************************************** limelight *****************************************************/


		double targetSeen = m_limetable->GetNumber("tv",0.0);
		double targetArea = m_limetable->GetNumber("ta",0.0);

		double targetOffsetAngle_Vertical = 0.0;
		if (targetSeen != 0.0) {
		  frc::SmartDashboard::PutNumber("Targ Area", targetArea);
		  if (targetArea > kMinTargetAreaPercent) {  // tv is true if there is a target detected
			//double targetOffsetAngle_Horizontal = m_limetable->GetNumber("tx",0.0);
			targetOffsetAngle_Vertical = m_limetable->GetNumber("ty",0.0);   
			//double targetSkew = m_limetable->GetNumber("ts",0.0);
			double targetWidth = m_limetable->GetNumber("tlong",0.0);

			frc::SmartDashboard::PutNumber("Targ Width", targetWidth);
			frc::SmartDashboard::PutNumber("Targ Vert", targetOffsetAngle_Vertical);
		  }
		}

		/********************************************** move stuff ********************************************/

		//m_turret->Set(ControlMode::PercentOutput, shooter_Y);  // temporary test

		if (auto_intake_button) {
			// bring balls in and index using photo eye
		} else { // enable manual control of intake
			m_intake.Set(-intake_Y * kIntakeSpeed);
		}
		
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

		// if (slow_gear_button_pressed) {speed_factor = kSlowSpeedFactor;}
		if (high_gear_button_presssed) {speed_factor = kFastSpeedFactor;}
		else {speed_factor = kSlowSpeedFactor;}
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
				m_robotDrive.ArcadeDrive(ScaleSpeed(robot_rel_Y, speed_factor), ScaleSpeed(robot_rel_X, speed_factor));
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

		double conveyer_speed = 0.0; 
		if (auto_shoot_button) {
			m_limetable->PutNumber("ledMode",3.0); // LED on
		} else {
			m_limetable->PutNumber("ledMode",1.0); // LED off
		}
		frc::SmartDashboard::PutNumber("targetSeen", targetSeen);
		if (targetSeen != 0.0 && auto_shoot_button) {
			frc::SmartDashboard::PutNumber("Seen", true);
			/* for tuning shooter
			shooter_speed_in_units = frc::SmartDashboard::GetNumber("shoot speed", 0.0);
			*/
		
			if (targetOffsetAngle_Vertical < -18) {
				shooter_speed_in_units = 23000;
			} else {
				shooter_speed_in_units = 12696.1 - 317.502 * targetOffsetAngle_Vertical;
			}
			frc::SmartDashboard::PutNumber("shoot speed 1", shooter_speed_in_units);

			m_shooter_star->Set(ControlMode::Velocity, -shooter_speed_in_units);
			double shooter_speed_error = m_shooter_star->GetClosedLoopError();
			frc::SmartDashboard::PutNumber("shooter err", shooter_speed_error);
			if (shooter_speed_error < kMaxShooterSpeedError) {
				// auto feed balls into shooter
				conveyer_speed = -kConveyerSpeed;
			} 
			//if (m_shooter_star->GetClosedLoopError < kShooterSpeedTolerance)
		} else { // no target; permit manual control
			frc::SmartDashboard::PutNumber("Seen", false);
			shooter_speed_in_units = kIdleShooterSpeed;
			m_shooter_star->Set(ControlMode::Velocity, -shooter_speed_in_units);
			if (conveyer_in_button) {
				conveyer_speed = -kConveyerSpeed;
			} else if (conveyer_out_button) {
				conveyer_speed = kConveyerSpeed;
			}
		}
		
		m_vert_conveyer.Set(conveyer_speed);
		frc::SmartDashboard::PutNumber("shoot speed 2", shooter_speed_in_units);
		frc::SmartDashboard::PutNumber("conveyer speed", conveyer_speed);
		

		// test digital sensors
		bool eye_value = eye_collector.Get();
		//frc::SmartDashboard::PutNumber("Eye", eye_value);

		// operate control panel
		if (spin_control_panel_button) {
			SpinThreeTimes();
		}
		if (spin_to_color_pressed) {
			SpinToColor(spin_to_color);
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

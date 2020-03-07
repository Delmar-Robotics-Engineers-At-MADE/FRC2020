
#include <iostream>
#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/SpeedControllerGroup.h>

#include <frc/controller/PIDController.h>

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
	Joystick * _joy = new Joystick(0);
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

		m_stick = new Joystick(k_joystick_pilot);
		m_stick_copilot = new Joystick(k_joystick_copilot);
		rotateToAngleRate = 0.0f;

		m_robotDrive.SetExpiration(0.1);
	}

	void TeleopPeriodic() {
		/* get gamepad axis */
		double leftYstick = _joy->GetY();
		double motorOutput = m_shooter_star->GetMotorOutputPercent();

		/* while button1 is held down, closed-loop on target velocity */
		if (_joy->GetRawButton(1)) {
        	/* Speed mode */
			/* Convert 500 RPM to units / 100ms.
			 * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
			// MJS: Falcon sensor reports 2048 units/rev

			double targetVelocity_UnitsPer100ms = leftYstick * 6000.0 * 2048 / 600;
        	m_shooter_star->Set(ControlMode::Velocity, targetVelocity_UnitsPer100ms); 

        } else {
			/* Percent voltage mode */
			m_shooter_star->Set(ControlMode::PercentOutput, leftYstick);
		}
	}
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include "ctre/Phoenix.h"
#include <frc/controller/PIDController.h>

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
class Robot : public frc::TimedRobot {
  WPI_TalonSRX m_leftfront{1};
  WPI_TalonSRX m_leftrear{2};
  WPI_TalonSRX m_rightfront{7};
  WPI_TalonSRX m_rightrear{8};
  WPI_TalonFX m_shooter_star{20}; // 20 is starboard, 21 is port
  WPI_TalonFX m_shooter_port{21}; // 20 is starboard, 21 is port
  frc::SpeedControllerGroup m_left{m_leftfront, m_leftrear};
  frc::SpeedControllerGroup m_right{m_rightfront, m_rightrear};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};
  //frc::DifferentialDrive m_robotDrive{m_right, m_left};
  frc::Joystick m_stick{0};

  static constexpr double kP = -5.0;
  static constexpr double kI = -0.02;
  static constexpr double kD = -2.0;
  frc2::PIDController m_pidController{kP, kI, kD};

 public:

  void RobotInit(){
    		/* factory default values */
		m_shooter_star.ConfigFactoryDefault();
		m_shooter_port.ConfigFactoryDefault();

		/* set up followers */
		m_shooter_port.Follow(m_shooter_star);
    m_shooter_star.SetInverted(true);
    m_shooter_port.SetInverted(false);

   }

  void TeleopPeriodic() {
    // Drive with arcade style
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetX());
    //m_right.Set(m_stick.GetY());
    switch(m_stick.GetPOV(0)) {
      case 0: m_shooter_star.Set(-0.25); break;
      case 90: m_shooter_star.Set(-0.5); break;
      case 180: m_shooter_star.Set(-0.75); break;
      case 270: m_shooter_star.Set(-1); break;
      default: m_shooter_star.Set(0);
    }
  }

};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

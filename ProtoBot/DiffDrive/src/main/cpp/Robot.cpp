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

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
class Robot : public frc::TimedRobot {
  WPI_TalonSRX m_leftfront{1};
  WPI_TalonSRX m_leftrear{2};
  WPI_TalonSRX m_rightfront{7};
  WPI_TalonSRX m_rightrear{8};
  frc::SpeedControllerGroup m_left{m_leftfront, m_leftrear};
  frc::SpeedControllerGroup m_right{m_rightfront, m_rightrear};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};
  //frc::DifferentialDrive m_robotDrive{m_right, m_left};
  frc::Joystick m_stick{0};

 public:
  void TeleopPeriodic() {
    // Drive with arcade style
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetX());
    //m_right.Set(m_stick.GetY());
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

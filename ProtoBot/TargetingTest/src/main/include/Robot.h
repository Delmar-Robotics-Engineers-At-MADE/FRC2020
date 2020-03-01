/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/Joystick.h>
#include <frc/Spark.h>
#include <frc/Timer.h>
#include <frc/Servo.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/livewindow/LiveWindow.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/controller/PIDController.h>

#include <frc/SpeedControllerGroup.h>
#include "ctre/Phoenix.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  Robot() {    
      m_robotDrive.SetExpiration(0.1);    
      m_timer.Start();  
    }

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  std::shared_ptr<NetworkTable> m_limetable;  // for LimeLight

  // for PID
  static constexpr double kP = 0.02;
  static constexpr double kI = 0.01;
  static constexpr double kD = 0.001;
  static constexpr double kPr = 0.08;
  static constexpr double kIr = 0.04;
  static constexpr double kDr = 0.0;
  frc2::PIDController m_pidController{kP, kI, kD};
  frc2::PIDController m_pidControllerRange{kPr, kIr, kDr};

  // Robot drive system  
  WPI_TalonSRX m_leftfront{1};
  WPI_TalonSRX m_leftrear{2};
  WPI_TalonSRX m_rightfront{7};

  WPI_TalonSRX m_rightrear{8}; 
  WPI_TalonSRX m_shooter{6};
  frc::SpeedControllerGroup m_left{m_leftfront, m_leftrear};
  frc::SpeedControllerGroup m_right{m_rightfront, m_rightrear};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};
  frc::Joystick m_stick{0};  
  frc::Joystick m_stick_copilot{1};  
  frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();  
  frc::Timer m_timer;
  frc::Servo m_limeServo{2};
  double m_limeServoAngle = 90.0;
  bool m_okToPursue = false;





};

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
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
// #include <frc/Spark.h>
#include <frc/DriverStation.h>
#include <frc/controller/PIDController.h>
#include "AHRS.h"
#include <frc/SpeedControllerGroup.h>
#include "ctre/Phoenix.h"
#include <frc/drive/DifferentialDrive.h>

using namespace frc;

class Robot : public TimedRobot /*, public PIDOutput */ {  // MJS: modified for new PID framework
 public:
  void RobotInit() override;
  //void RobotInit2() ;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  //void PIDWrite(double output) override;

 private:
    // Channels for the wheels
    const static int frontLeftChannel = 2;
    const static int rearLeftChannel = 3;
    const static int frontRightChannel = 1;
    const static int rearRightChannel = 0;

    WPI_TalonSRX m_leftfront{1};
    WPI_TalonSRX m_leftrear{2};
    WPI_TalonSRX m_rightfront{7};
    WPI_TalonSRX m_rightrear{8};
    WPI_TalonSRX m_shooter{6};
    frc::SpeedControllerGroup m_left{m_leftfront, m_leftrear};
    frc::SpeedControllerGroup m_right{m_rightfront, m_rightrear};
    frc::DifferentialDrive m_robotDrive{m_left, m_right};

    const static int joystickChannel = 0;

    Joystick *stick;          // only joystick
    AHRS *ahrs;

    static constexpr double kP = 0.008;
    static constexpr double kI = 0.001;
    static constexpr double kD = 0.0005;
    frc2::PIDController m_pidController{kP, kI, kD};
    frc::Timer m_timer;

    double rotateToAngleRate;           // Current rotation rate
};

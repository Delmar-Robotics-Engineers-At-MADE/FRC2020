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

 private:
    double TrimSpeed (double s, double max);
    double ScaleSpeed (double s, double scale);
    double ConvertRadsToDegrees (double rads);

    // Channels for the wheels
    // const static int frontLeftChannel = 2;
    // const static int rearLeftChannel = 3;
    // const static int frontRightChannel = 1;
    // const static int rearRightChannel = 0;

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

    // shooter
    TalonFX * m_shooter_star = new TalonFX(15); // 15 is starboard, 0 is port
    TalonFX * m_shooter_port = new TalonFX(0); // 15 is starboard, 0 is port

    Joystick *m_stick, *m_stick_copilot;          // only joystick
    AHRS *ahrs;

    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;

    frc2::PIDController *m_pidController;
    frc::Timer m_timer;

    double rotateToAngleRate;           // Current rotation rate
    double speed_factor = 0.5;

    // joystick channels
    const static int k_joystick_pilot = 0;
    const static int k_joystick_copilot = 1;

};


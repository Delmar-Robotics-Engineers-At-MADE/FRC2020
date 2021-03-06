/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <wpi/raw_ostream.h>
#include <algorithm>  // for min and max

#include <frc/smartdashboard/SmartDashboard.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/controller/PIDController.h>

#define MOTOR_SCALE 1.7
#define TARGET_TOLERANCE_V 4
#define TARGET_TOLERANCE_H 8
#define LIMELIGHT_ANGLE_DEFAULT 65
#define SPEED_ROTATE 0.25
#define SPEED_PURSUE 0.25
#define MIN_TARGET_AREA_PERCENT 0.1
#define MAX_TARGET_AREA_PERCENT 1
using namespace std;

void Robot::RobotInit() {
  m_limetable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  m_robotDrive.SetRightSideInverted(true);
  //m_left.SetInverted(true);
  //m_right.SetInverted(true);

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  m_pidController.SetTolerance(4);
  m_pidControllerRange.SetTolerance(4);

  
  // breaking mode and other config of motor controller
  m_shooter.ConfigFactoryDefault();
  m_shooter.SetNeutralMode(NeutralMode::Coast);
  m_shooter.ConfigNeutralDeadband(0.25, 0); // maximum deadband is 25%
  m_shooter.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeoutMs);

  wpi::outs() << "hello from RobotInit\n";
  printf ("hello with printf\n");
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
    m_timer.Reset();
    m_timer.Start();
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
    // Drive for 2 seconds
    if (m_timer.Get() < 2.0) {
      m_robotDrive.TankDrive(0.5, 0); // left only
    } else if (m_timer.Get() < 4.0) {
      m_robotDrive.TankDrive(0, 0.5); // right only
    } else if (m_timer.Get() < 6.0) {
      // Drive forwards half speed
      m_robotDrive.ArcadeDrive(0.5, 0.0);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0);
    }
  }
}

void Robot::TeleopPeriodic() {

  m_shooter.Set(m_stick.GetRawAxis(5));

  bool ok_to_pursue_button_presssed = m_stick.GetRawButton(2);
  bool not_ok_to_pursue_button_presssed = m_stick.GetRawButton(3);
  bool shoot_button_pressed = m_stick.GetRawButton(4);

  if (ok_to_pursue_button_presssed) {
    m_okToPursue = true;
    frc::SmartDashboard::PutString("DB/String 1", "pursue enabled");
  } else if (not_ok_to_pursue_button_presssed) {
    m_okToPursue = false;
    frc::SmartDashboard::PutString("DB/String 1", "pursue disabled");
  }

  if (shoot_button_pressed) {
    m_limetable->PutNumber("ledMode",3); // LED on
    m_shooter.Set(ControlMode::Velocity, -1000 * 6.1); // 4048? units/100ms * 1min/600 * 1000 RPM... m_shooter.Set(-1); 
    frc::SmartDashboard::PutString("DB/String 0", "limelight cv mode");
    if (m_shooter.GetClosedLoopError() > SHOOTER_TOLERANCE) {
      frc::SmartDashboard::PutString("Shooter State", "spinning up");
    } else {
      frc::SmartDashboard::PutString("Shooter State", "ready to shoot");
    }
  } else {
    m_limetable->PutNumber("ledMode",1); // LED off
    // m_limetable->PutNumber("stream",1.0);  // secondary camera in PIP
    frc::SmartDashboard::PutString("DB/String 0", "limelight driver mode");
    frc::SmartDashboard::PutString("Shooter State", "off");
  }

  double speed_left = 0.0;
  double speed_right = 0.0;

  double targetSeen = m_limetable->GetNumber("tv",0.0);
  double targetArea = m_limetable->GetNumber("ta",0.0);
  frc::SmartDashboard::PutNumber("Target Area", targetArea);

  if (targetSeen != 0.0 && targetArea > MIN_TARGET_AREA_PERCENT) {  // tv is true if there is a target detected
    double targetOffsetAngle_Horizontal = m_limetable->GetNumber("tx",0.0);
    double targetOffsetAngle_Vertical = m_limetable->GetNumber("ty",0.0);   
    double targetSkew = m_limetable->GetNumber("ts",0.0);

    // vertical elevation by servo
    if (targetOffsetAngle_Vertical > TARGET_TOLERANCE_V) {
      m_limeServoAngle -= 1;
    } else if (targetOffsetAngle_Vertical < -TARGET_TOLERANCE_V) {
      m_limeServoAngle += 1;
    }
    m_limeServoAngle = std::min(m_limeServoAngle, 180.0);
    m_limeServoAngle = std::max(m_limeServoAngle, 0.0);
    //m_limeServoAngle = 90.0 - targetOffsetAngle_Vertical;
    m_limeServo.SetAngle(m_limeServoAngle);

    // drive toward target

    m_pidControllerRange.SetSetpoint(5);
    double pursue_speed = m_pidControllerRange.Calculate(targetArea);
    frc::SmartDashboard::PutNumber("DB/Slider 0", pursue_speed);
    if (m_okToPursue && targetArea <= MAX_TARGET_AREA_PERCENT && targetArea > MIN_TARGET_AREA_PERCENT) { // pursue target until it's bigger
      speed_right = pursue_speed; speed_left = pursue_speed;
    }

    // if (m_okToPursue && targetArea <= 10.0 && targetArea > 5) { // pursue target until it's bigger
    //   speed_left = SPEED_PURSUE; speed_right = SPEED_PURSUE;
    // } else if (m_okToPursue && targetArea > 10.0) { // back up!
    //   speed_left = -SPEED_PURSUE; speed_right = -SPEED_PURSUE;
    // }

    // center on target
    
    m_pidController.SetSetpoint(0);
    double diff_speed = m_pidController.Calculate(targetOffsetAngle_Horizontal);
    speed_left -= diff_speed;
    speed_right += diff_speed;

    // if (targetOffsetAngle_Horizontal > TARGET_TOLERANCE_H) {
    //   speed_left += SPEED_ROTATE;
    //   speed_right -= SPEED_ROTATE;
    // } else if (targetOffsetAngle_Horizontal < -TARGET_TOLERANCE_H) {
    //   speed_left -= SPEED_ROTATE;
    //   speed_right += SPEED_ROTATE;
    // }

    // move robot
    m_robotDrive.TankDrive(speed_left, speed_right, false);
    // m_robotDrive.TankDrive(0.2, 0.2, false);

    // check to see if we're within tolerance, and if so, reset
    if (m_pidController.AtSetpoint()) {
      m_pidController.Reset(); // clears out integral state, etc
    }
    if (m_pidControllerRange.AtSetpoint()) {
      m_pidControllerRange.Reset(); // clears out integral state, etc
    }

  } else {  // no vision target seen
    //   leave vertical alone, was... m_limeServo.SetAngle(LIMELIGHT_ANGLE_DEFAULT);
    // Drive with arcade style (use right stick)    
    m_robotDrive.ArcadeDrive(-m_stick.GetY()/MOTOR_SCALE, m_stick.GetX()/MOTOR_SCALE); // MJS: not so fast
  }

}

void Robot::TestPeriodic() {}
void Robot::TeleopInit() {
  m_limetable->PutNumber("camMode",0.0); // camera in normal CV mode
  m_limetable->PutNumber("stream",0.0);  // secondary camera side-by-side
  /* set closed loop gains in slot0 */
  m_shooter.Config_kF(kPIDLoopIdx, 0.1097, kTimeoutMs);
  m_shooter.Config_kP(kPIDLoopIdx, 0.22, kTimeoutMs);
  m_shooter.Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
  m_shooter.Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

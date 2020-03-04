/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <frc/util/color.h>

#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

#define MIN_CONFIDENCE 0.85 // for color match

/**
 * This is a simple example to show how the REV Color Sensor V3 can be used to
 * detect various colors.
 */
class Robot : public frc::TimedRobot {
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  rev::ColorSensorV3 m_colorSensor{i2cPort};

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  rev::ColorMatch m_colorMatcher;

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
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

  // state machine for counting rotations
  enum States {
	kUnknownState = 0,
  kOffStartingColor,
  kOnStartingColor
  };
  int wheel_state = kUnknownState;
  frc::Color starting_color = kNoColor;
  int half_rotation_count = 0;

 public:
  void RobotInit() {
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
  }
  void TeleopInit() {
    wheel_state = kUnknownState;
    starting_color = kNoColor;
    half_rotation_count = 0;
  }
  void RobotPeriodic() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    frc::Color detectedColor = m_colorSensor.GetColor();

    /**
     * Run the color match algorithm on our detected color
     */
    std::string colorString;
    double confidence = 0.0;
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    if (matchedColor == kBlueTarget) {
      colorString = "Blue";
    } else if (matchedColor == kRedTarget) {
      colorString = "Red";
    } else if (matchedColor == kGreenTarget) {
      colorString = "Green";
    } else if (matchedColor == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("Confidence", confidence);
    frc::SmartDashboard::PutString("Detected Color", colorString);
  
    switch (wheel_state) {
      case kUnknownState: 
        if (confidence > MIN_CONFIDENCE) {
          wheel_state = kOnStartingColor;
          starting_color = matchedColor;
        }
        break;
      case kOnStartingColor:
        if (confidence > MIN_CONFIDENCE && !(matchedColor == starting_color)) {
          wheel_state = kOffStartingColor;
        }
        break;
      case kOffStartingColor:
        if (confidence > MIN_CONFIDENCE && matchedColor == starting_color) {
          wheel_state = kOnStartingColor;
          half_rotation_count++;
        }
        break;
    }
    
    frc::SmartDashboard::PutNumber("Wheel state", wheel_state);
    frc::SmartDashboard::PutNumber("Turn count", half_rotation_count/2);

  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WOFConstants;
import frc.robot.util.FMS;
import frc.robot.util.RobotType;
import frc.robot.util.WOFColor;

import java.util.Map;

public class WheelOfFortune extends SubsystemBase {
  /**
   * Creates a new WheelOfFortune.
   */
  private WPI_TalonSRX liftTalon;
  private WPI_TalonSRX spinTalon;
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher = new ColorMatch();
  //private SuppliedValueWidget<Boolean> colorWidget = Shuffleboard.getTab("WOF").addBoolean("Color", () -> true);

  public WheelOfFortune() {
    // m_colorSensor = RobotType.isPracticeBot ? null : new ColorSensorV3(I2C.Port.kOnboard);
    m_colorSensor = null;

    liftTalon = new WPI_TalonSRX(7);
    spinTalon = new WPI_TalonSRX(8);
    // for (Color color : WOFConstants.COLOR_MAP.keySet()) {
    //   m_colorMatcher.addColorMatch(color);
    // }
  }

  public void wofUp(){
    liftTalon.set(0.5);
  }

  public void wofDown(){
    liftTalon.set(-0.5);
  }

  public void wofStop(){
    liftTalon.set(0);
  }

  public void wofSpin(double speed){
    spinTalon.set(speed);
  }

  public void wofSpinStop(){
    spinTalon.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    String gameDataColor = FMS.getColorFromGameData();
    if (gameDataColor != null) {
      SmartDashboard.putString("Target Color", gameDataColor);
    }
    if (m_colorSensor != null) {
      WOFColor wofColor = readColor();
      if (wofColor != null && wofColor.getColor() != null) {
        SmartDashboard.putString("Detected Color", wofColor.getColor());
        SmartDashboard.putNumber("Confidence", wofColor.getConfidence());
      }
    }
    //colorWidget.withProperties(Map.of("colorWhenTrue", colorString));
  }

  public WOFColor readColor() {
    Color detectedColor = m_colorSensor.getColor();
    if (detectedColor == null) {
      return null;
    }
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    return new WOFColor(WOFConstants.COLOR_MAP.get(match.color), match.confidence);
  }
}

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
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  //private SuppliedValueWidget<Boolean> colorWidget = Shuffleboard.getTab("WOF").addBoolean("Color", () -> true);

  public WheelOfFortune() {
    liftTalon = new WPI_TalonSRX(7);
    spinTalon = new WPI_TalonSRX(8);
    for (Color color : WOFConstants.COLOR_MAP.keySet()) {
      m_colorMatcher.addColorMatch(color);
    }
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

  public void wofSpin(){
    spinTalon.set(0.3);
  }

  public void wofSpinStop(){
    spinTalon.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Detected Color", readColor());
  }

  public String readColor() {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    SmartDashboard.putNumber("Confidence", match.confidence);
    //colorWidget.withProperties(Map.of("colorWhenTrue", colorString));
    return WOFConstants.COLOR_MAP.get(match.color);
  }
}

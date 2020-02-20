/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.revrobotics.ColorMatch;
// import com.revrobotics.ColorMatchResult;
// import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WOFConstants;

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
  // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  // private final ColorMatch m_colorMatcher = new ColorMatch();
  // private String lastColor = "NONE";

  public WheelOfFortune() {
    liftTalon = new WPI_TalonSRX(7);
    spinTalon = new WPI_TalonSRX(8);
    // m_colorMatcher.addColorMatch(WOFConstants.BLUE_TARGET);
    // m_colorMatcher.addColorMatch(WOFConstants.GREEN_TARGET);
    // m_colorMatcher.addColorMatch(WOFConstants.RED_TARGET);
    // m_colorMatcher.addColorMatch(WOFConstants.YELLOW_TARGET);

    // // Create Boolean widget that displays the color
    // colorWidget = Constants.WOFConstants.COLOR_MAP.add("Color", false);
    // colorWidget.withPosition(0, 4);
    // colorWidget.withProperties(COLOR_MAP.of("colorWhenFalse", "black"));
    // colorWidgetEntry = colorWidget.getEntry();
  }

  public void wofUp(){
    liftTalon.set(0.3);
  }

  public void wofDown(){
    liftTalon.set(-0.3);
  }

  public void wofStop(){
    liftTalon.set(0);
  }

  public void wofSpin(){
    spinTalon.set(1);
  }

  public void wofSpinStop(){
    spinTalon.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Color detectedColor = m_colorSensor.getColor();
    // ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    // String colorString = WOFConstants.COLOR_MAP.get(match.color);
    // if (!lastColor.equals(colorString)) {
    //   SmartDashboard.putString("Last Color Change", String.format("Detected color change from %s to %s.", lastColor, colorString));
    //   //fire change color condition
    // }
    // lastColor = colorString;

    // SmartDashboard.putNumber("Confidence", match.confidence);
    // SmartDashboard.putString("Detected Color", colorString);

    // Color color = robotContainer.readColor();
    
    // if (!colorSet) {
    //   // Choose "true" color based on color of wheel required for Position Control
    //   colorWidget.withProperties(Map.of("colorWhenTrue", color.value));
    //   colorWidgetEntry.setBoolean(true);
    //   colorSet = true;
  }
}

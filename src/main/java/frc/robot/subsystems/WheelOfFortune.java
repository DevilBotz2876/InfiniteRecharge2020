/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WheelOfFortune extends SubsystemBase {
  /**
   * Creates a new WheelOfFortune.
   */
  private WPI_TalonSRX liftTalon;
  private WPI_TalonSRX spinTalon;

  public WheelOfFortune() {
    liftTalon = new WPI_TalonSRX(7);
    spinTalon = new WPI_TalonSRX(8);
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
    spinTalon.set(0.5);
  }

  public void wofSpinStop(){
    spinTalon.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

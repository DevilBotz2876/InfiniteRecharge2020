/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /**
   * Creates a new Arm.
   */
//TODO add encoder/limit switch

  private WPI_TalonSRX armTalon;

  public Arm() {
    armTalon = new WPI_TalonSRX(6);

    TalonSRXConfiguration allConfigs = new TalonSRXConfiguration();
    armTalon.configAllSettings(allConfigs);
  }

  public void armUp(){
    armTalon.set(0.4);
  }

  public void armDown(){
    armTalon.set(-0.25);
  }

  public void armStop(){
    armTalon.set(0.055);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

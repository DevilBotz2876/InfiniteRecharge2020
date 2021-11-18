/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climber extends SubsystemBase {

  private final WPI_TalonSRX climberTalon;

  /**
   * Creates a new Climber.
   */
  public Climber() {
    climberTalon = new WPI_TalonSRX(9);

    climberTalon.setSensorPhase(true);
  }

  public void startWinding() {
    climberTalon.set(0.2);
  }

  public void startRewinding() {
    climberTalon.set(-0.2);
  }

  public void stopWinding() {
    climberTalon.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}

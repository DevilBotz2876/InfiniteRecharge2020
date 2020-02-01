/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.subsystems;

import frc.robot.commands.ExampleCommand;
import frc.robot.subsystem.RobotSubsystem;

public class ExampleSubsystem extends RobotSubsystem {
  /**
   * Creates a new ExampleSubsystem.
   */
  public ExampleSubsystem() {
  super("ExampleSubsystem");
    addCommand(new ExampleCommand(this));
    addValue("ExampleValue", Math::random);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

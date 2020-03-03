/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimbToggle extends CommandBase {

  private final Climber climber;

  private boolean isClimbing = false;

  /**
   * Creates a new ClimbToggle.
   */
  public ClimbToggle(Climber subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = subsystem;
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isClimbing = !this.isClimbing;
    if (this.isClimbing) {
      climber.startWinding();
    } else {
      climber.stopWinding();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistanceOrTime extends CommandBase {
  private final DriveTrain m_drive;
  private final double m_distance, m_speed, desiredTime;
  private long startTime;

  /**
   * Creates a new DriveDistance.
   *
   * @param inches The number of inches the robot will drive
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   * @param desiredTime The time to wait before giving up and ending the command
   */
  public DriveDistanceOrTime(DriveTrain drive, double inches, double speed, double desiredTime) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    this.desiredTime = desiredTime;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_drive.resetEncoders();
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_speed, 0);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return m_drive.getAverageEncoderDistance() >= m_distance || (System.currentTimeMillis() - startTime) / 1000.0 >= desiredTime;
  }
}
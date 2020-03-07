/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class AutoDrive extends SequentialCommandGroup {

  public AutoDrive(DriveTrain drive, Intake intake) {
    addCommands(
            new DriveDistance(drive, Constants.AutoConstants.DISTANCE_TO_GOAL, 0.5),
            new BallOutTimed(intake, 2),
            new DriveDistance(drive, Constants.AutoConstants.DISTANCE_TO_GOAL, -0.5),
            new DriveRotate(drive, Constants.AutoConstants.ROTATE_ANGLE, 0.7));
  }
}

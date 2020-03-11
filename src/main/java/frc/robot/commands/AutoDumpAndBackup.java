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

public class AutoDumpAndBackup extends SequentialCommandGroup {

  public AutoDumpAndBackup(DriveTrain drive, Intake intake) {
    addCommands(
            new DriveDistanceOrTime(drive, Constants.AutoConstants.DISTANCE_TO_GOAL, 0.5, Constants.AutoConstants.DRIVE_TIMEOUT),
            //TODO: Add arm down to position -400 then arm up to position 250
            new BallOutTimed(intake, 2),
            new DriveDistanceOrTime(drive, Constants.AutoConstants.DISTANCE_TO_GOAL, -0.5, Constants.AutoConstants.DRIVE_TIMEOUT),
            new DriveRotate(drive, Constants.AutoConstants.ROTATE_ANGLE, 0.7));
  }
}

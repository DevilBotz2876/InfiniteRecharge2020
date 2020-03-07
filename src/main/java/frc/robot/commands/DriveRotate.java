/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveRotate extends CommandBase {
    private final DriveTrain m_drive;
    private double initialRotation;
    private final double degrees, rotationSpeed;

    public DriveRotate(DriveTrain drive, double degrees, double rotationSpeed) {
        this.degrees = degrees;
        this.rotationSpeed = rotationSpeed;
        m_drive = drive;
    }

    @Override
    public void initialize() {
        m_drive.resetGyro();
        initialRotation = m_drive.getAngle().getDegrees();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.arcadeDrive(0, rotationSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return m_drive.getAngle().getDegrees() >= initialRotation + degrees - 66;
    }
}
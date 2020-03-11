/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmPosition extends CommandBase {
    /**
     * Creates a new ArmDown.
     */
    private final Arm arm;
    private final double position, desiredTime;
    private long startTime;

    public ArmPosition(Arm subsystem, double position, double desiredTime) {
        // Use addRequirements() here to declare subsystem dependencies.
        arm = subsystem;
        assert position != 0;
        this.position = position;
        this.desiredTime = desiredTime;
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.resetArmEncoder();
        startTime = System.currentTimeMillis();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (position > 0) {
            arm.armUp();
        } else {
            arm.armDown();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.armStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (position > 0 ? arm.getEncoderDistance() >= position : arm.getEncoderDistance() <= position) || (System.currentTimeMillis() - startTime) / 1000.0 >= desiredTime;
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelOfFortune;

public class WOFSpinNextColor extends CommandBase {
    /**
     * Creates a new WOFSpin.
     */
    private final WheelOfFortune wof;

    public WOFSpinNextColor(WheelOfFortune subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        wof = subsystem;
        addRequirements(wof);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        wof.setSpinState(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!wof.getSpinState()) {
            cancel();
            return;
        }
        wof.wofSpin();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelOfFortune;
import frc.robot.util.ColorOffset;
import frc.robot.util.FMS;
import frc.robot.util.WOFColor;

public class WOFSpinToColor extends CommandBase {
    /**
     * Creates a new WOFSpin.
     */
    private final WheelOfFortune wof;
    private String targetColor, currentColor;

    public WOFSpinToColor(WheelOfFortune subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        wof = subsystem;
        String color = FMS.getColorFromGameData();
        if (color != null) {
            targetColor = ColorOffset.getOffsettedColor(color);
        } else {
            targetColor = null;
        }
        addRequirements(wof);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        wof.wofSpin(0.15);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        WOFColor wofColor = wof.readColor();
        if (wofColor.getConfidence() > .95) {
            currentColor = wofColor.getColor();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wof.wofSpinStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return targetColor == null || currentColor.equals(targetColor);
        // return false;
    }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelOfFortune;

public class WOFSpinForSameColor extends CommandBase {
    /**
     * Creates a new WOFSpin.
     */
    private final WheelOfFortune wof;
    private String lastColor, initialColor;
    private int currentTimes;
    private final int times;

    public WOFSpinForSameColor(WheelOfFortune subsystem, int times) {
        // Use addRequirements() here to declare subsystem dependencies.
        wof = subsystem;
        this.times = times;
        addRequirements(wof);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialColor = wof.readColor().getColor();
        lastColor = initialColor;
        wof.wofSpin(0.3);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        String color = wof.readColor().getColor();
        if (!lastColor.equals(color) && color.equals(initialColor)) {
            currentTimes++;
        }
        lastColor = color;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wof.wofSpinStop();
        currentTimes = 0;
        lastColor = null;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return currentTimes == times;
    }
}
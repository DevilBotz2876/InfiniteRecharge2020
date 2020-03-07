package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTimed extends CommandBase {
    /**
     * Creates a new AutoDrive.
     */
    private final DriveTrain m_drive;
    private final double desiredTime, speed;
    private long startTime;

    public DriveTimed(DriveTrain drive, double desiredTime, double speed) {
        m_drive = drive;
        this.desiredTime = desiredTime;
        this.speed = speed;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.arcadeDrive(speed, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime) / 1000.0 >= desiredTime;
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.ArmDown;
import frc.robot.commands.ArmStop;
import frc.robot.commands.ArmUp;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.BallIn;
import frc.robot.commands.BallOut;
import frc.robot.commands.BallStop;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.WOFDown;
import frc.robot.commands.WOFSpin;
import frc.robot.commands.WOFSpinForSameColor;
import frc.robot.commands.WOFSpinStop;
import frc.robot.commands.WOFSpinToColor;
import frc.robot.commands.WOFStop;
import frc.robot.commands.WOFUp;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.WheelOfFortune;
import frc.robot.util.XboxTrigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain drive = new DriveTrain();
  private final Intake intake = new Intake();
  private final Arm arm = new Arm();
  private final WheelOfFortune wof = new WheelOfFortune();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final AutoDrive autoCommand = new AutoDrive(drive);

  private final XboxController controller = new XboxController(0);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureAutonomous();

    // Configure the button bindings
    configureButtonBindings();

    drive.setDefaultCommand(new DriveCommand(drive, () -> -controller.getY(GenericHID.Hand.kLeft),
        () -> -controller.getX(GenericHID.Hand.kRight)));

    intake.setDefaultCommand(new BallStop(intake));
    arm.setDefaultCommand(new ArmStop(arm));

    SmartDashboard.putData(intake);
    SmartDashboard.putData(arm);
    SmartDashboard.putData(drive);

    SmartDashboard.putData(new WOFSpinForSameColor(wof, 6));
    SmartDashboard.putData(new WOFSpinToColor(wof, "YELLOW"));
    SmartDashboard.putData(new AutoDrive(drive));

     // Turn off LiveWindow telemetry. Consider doing/testing this out if we do not use it.
     // LiveWindow.disableAllTelemetry();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(controller, Button.kBumperRight.value).whenPressed(new BallIn(intake))
        .whenReleased(new BallStop(intake));

    new JoystickButton(controller, Button.kBumperLeft.value).whenPressed(new BallOut(intake))
        .whenReleased(new BallStop(intake));

    // does not work, uses buttons b and x instead
    new XboxTrigger(controller, Axis.kRightTrigger.value).whenPressed(new ArmUp(arm)).whenReleased(new ArmStop(arm));

    new XboxTrigger(controller, Axis.kLeftTrigger.value).whenPressed(new ArmDown(arm)).whenReleased(new ArmStop(arm));

    new JoystickButton(controller, Button.kY.value).whenPressed(new WOFUp(wof)).whenReleased(new WOFStop(wof));

    new JoystickButton(controller, Button.kA.value).whenPressed(new WOFDown(wof)).whenReleased(new WOFStop(wof));

    new JoystickButton(controller, Button.kX.value).whenPressed(new WOFSpin(wof)).whenReleased(new WOFSpinStop(wof));
  }

  public void makeAutoStraightCommand() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveTrainConstants.kV, DriveTrainConstants.kV, DriveTrainConstants.kA),
        DriveTrainConstants.DRIVE_KINEMATICS, 10);
    TrajectoryConfig config = new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO,
        TrajectoryConstants.MAX_ACCELERATION_AUTO)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    Trajectory straight2m = TrajectoryGenerator.generateTrajectory(
        // start at origin facing +X dir
        new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0)),
        // one waypoint straight ahead
        List.of(new Translation2d(1.0, 0.0)),
        // go 2m and end facing +X dir
        new Pose2d(new Translation2d(2.0, 0.0), new Rotation2d(0)),
        //
        config);

    var straightPathCommand = drive.createCommandForTrajectory(straight2m);
    autoChooser.setDefaultOption("Straight 2m", straightPathCommand);
  }

  private void makeAutoCurveCommand() {

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveTrainConstants.kV, DriveTrainConstants.kV, DriveTrainConstants.kA),
        DriveTrainConstants.DRIVE_KINEMATICS, 10);
    TrajectoryConfig config = new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO,
        TrajectoryConstants.MAX_ACCELERATION_AUTO)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    Trajectory newTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

    // //var straightTrajectory = loadTrajectory("Straight");
    // Transform2d transform = new Pose2d(0, 0,
    // Rotation2d.fromDegrees(0)).minus(straightTrajectory.getInitialPose());
    // Trajectory newTrajectory = straightTrajectory.transformBy(transform);

    var curvePathCommand = drive.createCommandForTrajectory(newTrajectory);
    autoChooser.addOption("Curvy", curvePathCommand);
    
  }

  public void loadTestTrajectory() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveTrainConstants.kV, DriveTrainConstants.kV, DriveTrainConstants.kA),
        DriveTrainConstants.DRIVE_KINEMATICS, 10);
    TrajectoryConfig config = new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO,
        TrajectoryConstants.MAX_ACCELERATION_AUTO)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    Trajectory newTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

    // //var straightTrajectory = loadTrajectory("Straight");
    // Transform2d transform = new Pose2d(0, 0,
    // Rotation2d.fromDegrees(0)).minus(straightTrajectory.getInitialPose());
    // Trajectory newTrajectory = straightTrajectory.transformBy(transform);

    var straightPathCommand = drive.createCommandForTrajectory(newTrajectory);
    autoChooser.setDefaultOption("Straight", straightPathCommand);
  }
    

  public void configureAutonomous() {
    makeAutoCurveCommand();
    makeAutoStraightCommand();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

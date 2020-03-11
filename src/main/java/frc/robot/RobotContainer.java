/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.RobotType;
import frc.robot.util.VideoStream;
import frc.robot.util.XboxTrigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain drive = new DriveTrain();
  private final Intake intake = new Intake();
  private final Arm arm = new Arm();
  private final WheelOfFortune wof = new WheelOfFortune();
  private final Climber climber = new Climber();

  private final XboxController controller = new XboxController(0);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putBoolean("isPracticeBot", RobotType.isPracticeBot);

    AutoDumpAndBackup autoCommand = new AutoDumpAndBackup(drive, intake);
    autoChooser.addOption("Dump And Back Up", autoCommand);
    DriveDistanceOrTime backupCommand = new DriveDistanceOrTime(drive, Constants.AutoConstants.DISTANCE_TO_GOAL, -0.5, Constants.AutoConstants.DRIVE_TIMEOUT);
    autoChooser.addOption("Back Up", backupCommand);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    VideoStream.create();

    // Configure the button bindings
    configureButtonBindings();

    drive.setDefaultCommand(new DriveCommand(drive, 
      () -> -controller.getY(GenericHID.Hand.kLeft),
      () -> -controller.getX(GenericHID.Hand.kRight)
      ));
    
    intake.setDefaultCommand(new BallStop(intake));
    arm.setDefaultCommand(new ArmStop(arm));

    SmartDashboard.putData(intake);
    SmartDashboard.putData(arm);
    SmartDashboard.putData(drive);

    SmartDashboard.putData(new WOFSpinForSameColor(wof, 6));
    SmartDashboard.putData(new WOFSpinToColor(wof));

    SmartDashboard.putData(new DriveTimed(drive, 2.5, 0.5));
    SmartDashboard.putData(new DriveDistanceOrTime(drive, Constants.AutoConstants.DISTANCE_TO_GOAL, 0.5, Constants.AutoConstants.DRIVE_TIMEOUT));
    SmartDashboard.putData(new AutoDumpAndBackup(drive, intake));
    SmartDashboard.putData("Backup", backupCommand);
    SmartDashboard.putData(new DriveRotate(drive, Constants.AutoConstants.ROTATE_ANGLE, .7));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // new JoystickButton(controller, Button.kBumperRight.value)
    //     .whenReleased(new BallIn(intake));
    new JoystickButton(controller, Button.kBumperRight.value)
    .whenPressed(new BallIn(intake))
    .whenReleased(new BallStop(intake));

    new JoystickButton(controller, Button.kBumperLeft.value)
        .whenPressed(new BallOut(intake))
        .whenReleased(new BallStop(intake));



    new XboxTrigger(controller, Axis.kRightTrigger.value)
        .whenPressed(new ArmUp(arm))
        .whenReleased(new ArmStop(arm));

    new XboxTrigger(controller, Axis.kLeftTrigger.value)
        .whenPressed(new ArmDown(arm))
        .whenReleased(new ArmStop(arm));

    new JoystickButton(controller, Button.kY.value)
        .whenPressed(new WOFUp(wof))
        .whenReleased(new WOFStop(wof));

    new JoystickButton(controller, Button.kA.value)
        .whenPressed(new WOFDown(wof))
        .whenReleased(new WOFStop(wof));

    new JoystickButton(controller, Button.kX.value)
        .whenPressed(new WOFSpin(wof))
        .whenReleased(new WOFSpinStop(wof));

    new JoystickButton(controller, Button.kB.value)
        .whenPressed(new ClimbOn(climber))
        .whenReleased(new ClimbOff(climber));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}

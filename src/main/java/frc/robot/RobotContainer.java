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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.WheelOfFortune;
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

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final AutoDrive autoCommand = new AutoDrive(drive, intake);

  private final XboxController controller = new XboxController(0);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
    SmartDashboard.putData(new DriveDistance(drive, 43.5, 0.5));
    SmartDashboard.putData(new AutoDrive(drive, intake));
    SmartDashboard.putData("Backup", new DriveDistance(drive, 42.5, -0.5));
    SmartDashboard.putData(new DriveRotate(drive, 90, .7));

    SmartDashboard.putData(new TurnToAngle(90, drive));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(controller, Button.kBumperRight.value)
        .whenPressed(new BallIn(intake))
        .whenReleased(new BallStop(intake));

    new JoystickButton(controller, Button.kBumperLeft.value)
        .whenPressed(new BallOut(intake))
        .whenReleased(new BallStop(intake));



        //does not work, uses buttons b and x instead
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
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
}

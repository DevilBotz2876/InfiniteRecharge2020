/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrainConstants.FEED_FORWARD;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.util.DriveTrainHelper;

public class DriveTrain extends SubsystemBase {
  /**
  * Creates a new DriveTrain.
  */
  
  //TODO figure out what these mean and/or if they matter
  // public static final double kMaxSpeed = 3.0; // meters per second
  // public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second 
  
  // private static final double kTrackWidth = 0.595; // meters
  
  // // 7in circumference wheel which is .1778m
  // // .1778/3.14 = .0566 diameter
  // // radius is .0556/2 = .0283 
  // // 0.158 was original value we had for radius
  // private static final double kWheelRadius = 0.0283; // meters
  // private static final int kEncoderResolution = 4096;
  
  private WPI_TalonSRX talonSRX4 = new WPI_TalonSRX(4);
  private WPI_TalonSRX talonSRX3 = new WPI_TalonSRX(3);
  private WPI_TalonSRX talonSRX2 = new WPI_TalonSRX(2);
  private WPI_TalonSRX talonSRX1 = new WPI_TalonSRX(1);
  
  private WPI_TalonSRX rightMaster = talonSRX2;
  private WPI_TalonSRX leftMaster = talonSRX4;
  private WPI_TalonSRX rightFollower = talonSRX1;
  private WPI_TalonSRX leftFollower = talonSRX3;
  
  private AHRS navx = new AHRS(SPI.Port.kMXP);
  
  DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
  
  //private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);
  
  private final DifferentialDriveOdometry differentialDriveOdometry;
  
  public DriveTrain() {
    
    differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    
    // https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html#follower
    rightFollower.follow(rightMaster);
    leftFollower.follow(leftMaster);
    
    // https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html#inverts
    // 1234 settings
    // rightMaster.setInverted(true);
    // rightFollower.setInverted(InvertType.FollowMaster);
    // leftMaster.setInverted(true);
    // leftFollower.setInverted(InvertType.FollowMaster);
    
    // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
    
    // 2876 settings
    leftMaster.setSensorPhase(true);
    rightMaster.setSensorPhase(true); 
    
    // 1234 settings
    // leftMaster.setSensorPhase(false);
    // rightMaster.setSensorPhase(true); 
    
    
    TalonSRXConfiguration allConfigs = new TalonSRXConfiguration();
    
    allConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    
    leftMaster.configAllSettings(allConfigs);
    rightMaster.configAllSettings(allConfigs);
    
    navx.reset();
    resetEncoders();
    
    // odometry = new DifferentialDriveOdometry(getAngle());
  }
  
  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0, 0, 0);
    rightMaster.setSelectedSensorPosition(0, 0, 0);
  }
  
  public void tankDrive(double leftValue, double rightValue) {
    differentialDrive.tankDrive(leftValue, rightValue);
  }

  public void arcadeDrive(double speed, double rotation) {
    differentialDrive.arcadeDrive(speed, rotation);
  }
  
  /**
  * Returns the angle of the robot as a Rotation2d.
  *
  * @return The angle of the robot.
  */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-navx.getAngle());
  }
  
  /**
  * Sets the desired wheel speeds.
  *
  * @param speeds The desired wheel speeds.
  */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    
  }
  
  /**
  * Drives the robot with the given linear velocity and angular velocity.
  *
  * @param xSpeed Linear velocity in m/s.
  * @param rot    Angular velocity in rad/s.
  */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    // var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    // setSpeeds(wheelSpeeds);
  }
  
  @Override
  public void periodic() {
    
  }
  
  

  /**
   * Controls the left and right side of the drive using Talon SRX closed-loop
   * velocity.
   * 
   * @param leftVelocity  left velocity in meters per second
   * @param rightVelocity right velocity in meters per second
   */
  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    var leftAccel = (leftVelocity - DriveTrainHelper.edgesPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity())) / .02;
    var rightAccel = (rightVelocity - DriveTrainHelper.edgesPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity())) / .02;
    
    var leftFeedForwardVolts = FEED_FORWARD.calculate(leftVelocity, leftAccel);
    var rightFeedForwardVolts = FEED_FORWARD.calculate(rightVelocity, rightAccel);

    leftMaster.set(
        ControlMode.Velocity, 
        DriveTrainHelper.metersPerSecToEdgesPerDecisec(leftVelocity), 
        DemandType.ArbitraryFeedForward,
        leftFeedForwardVolts / 12);

    rightMaster.set(
        ControlMode.Velocity,
        DriveTrainHelper.metersPerSecToEdgesPerDecisec(rightVelocity),
        DemandType.ArbitraryFeedForward,
        rightFeedForwardVolts / 12);
        
    differentialDrive.feed();
  }

  /**
   * Sets the drivetrain to zero velocity and rotation.
   */
  public void stop() {
    tankDriveVelocity(0, 0);
  }

  /**
   * Returns the heading of the robot in form required for odometry.
   *
   * @return the robot's heading in degrees, from -180 to 180 with positive value
   *         for left turn.
   */
  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360.0d) * -1.0d;
  }

  public Pose2d getCurrentPose() {
    return differentialDriveOdometry.getPoseMeters();
  }
  /**
  * Creates a command to follow a Trajectory on the drivetrain.
  * @param trajectory trajectory to follow
  * @return command that will run the trajectory
  */
  public Command createCommandForTrajectory(Trajectory trajectory) {
    return new RamseteCommand(
    trajectory,
    this::getCurrentPose,
    new RamseteController(TrajectoryConstants.RAMSETE_B, TrajectoryConstants.RAMSETE_ZETA),
    DriveTrainConstants.DRIVE_KINEMATICS,
    this::tankDriveVelocity,
    this);
  }
}

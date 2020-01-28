/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

   //TODO figure out what these mean and/or if they matter
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second 

  private static final double kTrackWidth = 0.595; // meters
  private static final double kWheelRadius = 0.158; // meters
  private static final int kEncoderResolution = 4096;

  private WPI_TalonSRX talonSRX4 = new WPI_TalonSRX(4);
  private WPI_TalonSRX talonSRX3 = new WPI_TalonSRX(3);
  private WPI_TalonSRX talonSRX2 = new WPI_TalonSRX(2);
  private WPI_TalonSRX talonSRX1 = new WPI_TalonSRX(1);

  private WPI_TalonSRX rightMaster = talonSRX2;
  private WPI_TalonSRX leftMaster = talonSRX3;
  private WPI_TalonSRX rightFollower = talonSRX1;
  private WPI_TalonSRX leftFollower = talonSRX4;

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  //TODO what should these be? should they be different? find out next time on
  private final PIDController leftPIDController = new PIDController(1, 0, 0);
  private final PIDController rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics kinematics
      = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry odometry;

  /**
   * Constructs a differential drive object.
   * Sets the encoder distance per pulse and resets the gyro.
   */
  public DriveTrain() {
  // https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html#follower
  rightFollower.follow(rightMaster);
  leftFollower.follow(leftMaster);

  // https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html#inverts
  // rightMaster.setInverted(true);
  // rightFollower.setInverted(InvertType.FollowMaster);
  // leftMaster.setInverted(true);
  // leftFollower.setInverted(InvertType.FollowMaster);

  // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
  leftMaster.setSensorPhase(false);
  rightMaster.setSensorPhase(true); 
    
    gyro.reset();

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    leftEncoder.reset();
    rightEncoder.reset();

    odometry = new DifferentialDriveOdometry(getAngle());
  }
  
  public void tankDrive(double leftValue, double rightValue) {
    differentialDrive.tankDrive(leftValue, rightValue);
  }
  
  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftOutput = leftPIDController.calculate(leftEncoder.getRate(),
        speeds.leftMetersPerSecond);
    double rightOutput = rightPIDController.calculate(rightEncoder.getRate(),
        speeds.rightMetersPerSecond);
    leftGroup.set(leftOutput);
    rightGroup.set(rightOutput);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /**
   * Updates the field-relative position.
   */
  public void updateOdometry() {
    odometry.update(getAngle(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }
  @Override
  public void periodic() {

  }
}

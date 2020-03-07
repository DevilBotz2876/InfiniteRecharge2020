/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

  private static final double TRACK_WIDTH = 0.595; // meters

  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(2);
  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(4);
  private WPI_TalonSRX rightFollower = new WPI_TalonSRX(1);
  private WPI_TalonSRX leftFollower = new WPI_TalonSRX(3);

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  //TODO what should these be? should they be different? find out next time on
  private final PIDController leftPIDController = new PIDController(1, 0, 0);
  private final PIDController rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics kinematics
      = new DifferentialDriveKinematics(TRACK_WIDTH);

  public DriveTrain() {
  // https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html#follower
  rightFollower.follow(rightMaster);
  leftFollower.follow(leftMaster);

  // https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html#inverts
  // 1234 settings
  rightMaster.setInverted(true);
  rightFollower.setInverted(InvertType.FollowMaster);
  leftMaster.setInverted(true);
  leftFollower.setInverted(InvertType.FollowMaster);


  // 2876 settings
  // rightMaster.setInverted(false);
  // rightFollower.setInverted(InvertType.FollowMaster);
  // leftMaster.setInverted(false);
  // leftFollower.setInverted(InvertType.FollowMaster);

  // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase

  // 2876 settings
  // leftMaster.setSensorPhase(true);
  // rightMaster.setSensorPhase(true); 

  // 1234 settings
  leftMaster.setSensorPhase(false);
  rightMaster.setSensorPhase(true); 
  
  
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

  /**
   * Calculates the distance traveled by the robot by reading encoder values
   * @return the linear distance traveled by the robot in inches
   */
  public double getAverageEncoderDistance() {
    double leftDistance = leftMaster.getSelectedSensorPosition() * (Constants.AutoConstants.kWheelDiameterInches * Math.PI / 4096);
    double rightDistance = rightMaster.getSelectedSensorPosition() * (Constants.AutoConstants.kWheelDiameterInches * Math.PI / 4096);
    return -((leftDistance + rightDistance) / 2);
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
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  @Override
  public void periodic() {
    double dist = getAverageEncoderDistance();
    SmartDashboard.putNumber("Distance", dist);
  }
}

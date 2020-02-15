/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.DriveTrainConstants.FEED_FORWARD;
import static frc.robot.Constants.DriveTrainConstants.DRIVE_KINEMATICS;

import java.util.Map;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Units;

/**
* The Constants class provides a convenient place for teams to hold robot-wide
* numerical or boolean constants. This class should not be used for any other
* purpose. All constants should be declared globally (i.e. public static). Do
* not put anything functional in this class.
*
* <p>
* It is advised to statically import this class (or one of its inner classes)
* wherever the constants are needed, to reduce verbosity.
*/
public final class Constants {
    
    public static final class WOFConstants {
        public static final Color BLUE_TARGET = ColorMatch.makeColor(0.121, 0.428, 0.451);
        public static final Color GREEN_TARGET = ColorMatch.makeColor(0.162, 0.588, 0.250);
        public static final Color RED_TARGET = ColorMatch.makeColor(0.528, 0.342, 0.130);
        public static final Color YELLOW_TARGET = ColorMatch.makeColor(0.317, 0.564, 0.119);
        public static final Map<Color, String> COLOR_MAP = Map.of(BLUE_TARGET, "BLUE", GREEN_TARGET, "GREEN",
        RED_TARGET, "RED", YELLOW_TARGET, "YELLOW");
    }
    
    public static final class DriveTrainConstants {

        // ctre magnetic encoder documenation provides this value
        // private static final int kEncoderResolution = 4096;
        public static final int EDGES_PER_ROTATION = 4096;

        //public static final double WHEEL_DIAMETER_INCHES = 6d;
        //public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
        //public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;
        
  // // 7in circumference wheel which is .1778m
  // // .1778/3.14 = .0566 diameter
  // // radius is .0556/2 = .0283 
  // // 0.158 was original value we had for radius
  // private static final double kWheelRadius = 0.0283; // meters

        public static final double WHEEL_DIAMETER_INCHES = 6d;

        public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;
        
        // Following values are from drive characterizatin tool
        //
        public static final double TRACK_WIDTH_METERS = 0.595;
        // Voltage needed to overcome the motor’s static friction. kS
        public static final double kS = 0.829;
        // Voltage needed to hold (or "cruise") at a given constant velocity. kV
        public static final double kV = 3.04;
        // Voltage needed to induce a given acceleration in the motor shaft. kA
        public static final double kA = 0.676;

        // ? 
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

        // May not use this if we use velocity mode on talons.
        public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);
        
    }
    
    public static final class TrajectoryConstants {
        
        // Max speed in meters per second
        public static final double MAX_SPEED_AUTO = 3;
        
        // Max acceleration in meters per second per second
        public static final double MAX_ACCELERATION_AUTO = 2;
        
        // Max voltage
        public static final double MAX_VOLTAGE_AUTO = 11;
        
        public static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT = 
        new DifferentialDriveVoltageConstraint(FEED_FORWARD, DRIVE_KINEMATICS, MAX_VOLTAGE_AUTO);
        
        // Baseline values for a RAMSETE follower in units of meters and seconds. Comes from wpilib docs
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;
    }
    
}

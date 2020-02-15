/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import static frc.robot.Constants.DriveTrainConstants.WHEEL_CIRCUMFERENCE_METERS;
import static frc.robot.Constants.DriveTrainConstants.EDGES_PER_ROTATION;;

/**
 * Add your docs here.
 */
public class DriveTrainHelper {


  /**
   * Converts from encoder edges to meters.
   * 
   * @param steps encoder edges to convert
   * @return meters
   */
  public static double edgesToMeters(int steps) {
    return (WHEEL_CIRCUMFERENCE_METERS / EDGES_PER_ROTATION) * steps;
  }

  /**
   * Converts from encoder edges per 100 milliseconds to meters per second.
   * @param stepsPerDecisec edges per decisecond
   * @return meters per second
   */
  public static double edgesPerDecisecToMetersPerSec(int stepsPerDecisec) {
    return edgesToMeters(stepsPerDecisec * 10);
  }

  /**
   * Converts from meters to encoder edges.
   * @param meters meters
   * @return encoder edges
   */
  public static double metersToEdges(double meters) {
    return (meters / WHEEL_CIRCUMFERENCE_METERS) * EDGES_PER_ROTATION;
  }

  /**
   * Converts from meters per second to encoder edges per 100 milliseconds.
   * @param metersPerSec meters per second
   * @return encoder edges per decisecond
   */
  public static double metersPerSecToEdgesPerDecisec(double metersPerSec) {
    return metersToEdges(metersPerSec) * .1d;
  }
}

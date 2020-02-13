/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

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
        public static final Color BLUE_TARGET = ColorMatch.makeColor(0, 0.5, 0.5);
        public static final Color GREEN_TARGET = ColorMatch.makeColor(0, 1, 0);
        public static final Color RED_TARGET = ColorMatch.makeColor(1, 0, 0);
        public static final Color YELLOW_TARGET = ColorMatch.makeColor(0.5, 0.5, 0);
        public static final Map<Color, String> COLOR_MAP = Map.of(BLUE_TARGET, "BLUE", GREEN_TARGET, "GREEN",
                RED_TARGET, "RED", YELLOW_TARGET, "YELLOW");
    }
}

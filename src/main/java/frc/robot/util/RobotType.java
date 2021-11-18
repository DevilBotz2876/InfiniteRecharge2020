package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class RobotType {

    public static final boolean isPracticeBot = isPracticeBot();

    private static boolean isPracticeBot() {
        System.out.println("********************* isPracticeBot ******************");
        return false;
        // DigitalInput dio = new DigitalInput(9);
        // return !dio.get();
    }
}
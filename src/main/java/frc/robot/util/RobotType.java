package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class RobotType {

    public static boolean isPracticeBot() {
        DigitalInput dio = new DigitalInput(9);
        boolean practiceBot = dio.get();
        dio.close();
        return practiceBot;
    }
}
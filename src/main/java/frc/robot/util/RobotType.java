package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class RobotType {
    
    public static boolean isPracticeBot() {
        DigitalInput practiceBotPort = new DigitalInput(9);
        return practiceBotPort.get();
    }
}

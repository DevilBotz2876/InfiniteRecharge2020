package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class FMS {

    public static String getColorFromGameData() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0) {
            for (String colorString : Constants.WOFConstants.COLOR_MAP.values()) {
                if (gameData.charAt(0) == colorString.charAt(0)) {
                    return colorString;
                }
            }
        }
        return null;
    }

}

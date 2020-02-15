package frc.robot.util;

public class ColorOffset {

    public static String getOffsettedColor(String originalColor) {
        switch (originalColor) {
            case "BLUE":
                return "RED";
            case "GREEN":
                return "YELLOW";
            case "RED":
                return "BLUE";
            case "YELLOW":
                return "GREEN";
            default:
                return null;
        }
    }

}

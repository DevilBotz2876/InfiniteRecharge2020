package frc.robot.util;

public class WOFColor {
    private String color;
    private double confidence;

    public WOFColor(String color, double confidence) {
        this.color = color;
        this.confidence = confidence;
    }

    public String getColor() {
        return color;
    }

    public double getConfidence() {
        return confidence;
    }
}

package frc.Mechanisms;

public class CatzMathUtils {
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
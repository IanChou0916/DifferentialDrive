
package frc.lib.util;

public class Conversions {
    // useful conversions for robots 6998.
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
    public static double metersToInches(double meters) {
        return meters / 0.0254;
    }
    public static double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180.0;
    }
    public static double radiansToDegrees(double radians) {
        return radians * 180.0 / Math.PI;
    }
    public static double degreesToCanCoder(double degrees, double gearRatio) {
        return degrees / 360.0 / (gearRatio * 4096.0);
    }
    public static double canCoderToDegrees(double canCoder) {
        return canCoder * (360.0 /1.0);
    }
    public static double degreesToFalcon(double degrees) {
        return degrees / 360.0;
    }
    public static double falconToDegrees(double falcon) {
        return falcon * 360.0 / 2048.0;
    }
    public static double degreesToNeo(double degrees, double gearRatio) {
        return degrees / 360.0 / (gearRatio * 42.0);
    }
    public static double neoToDegrees(double neo,double gearRatio) {
        return neo * (360.0/(gearRatio * 42.0));
    }
    public static double falconToRPM(double velocitycounts, double gearRatio) {
        return velocitycounts * 600.0 / (gearRatio * 2048.0);
    }
    public static double RPMToFalcon(double RPM, double gearRatio) {
        return RPM * (gearRatio * 2048.0) / 600.0;
    }
    public static double falconToMeters(double positioncounts,double circumference, double gearRatio) {
        return positioncounts * circumference / (gearRatio * 2048.0);
    }
    public static double metersToFalcon(double meters, double circumference, double gearRatio) {
        return meters * (gearRatio * 2048.0) / circumference;
    }
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        return velocitycounts * circumference / (gearRatio * 2048.0);
    }
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = (velocity*60) /circumference;
        double wheelVelocity =RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;

    }
}

/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.math;

/**
 * Class that manages falcon conversions
 * Also manages conversions for other things
 * 
 * Copied over and modified from
 * https://github.com/frc1678/C2022/blob/main/src/main/java/com/lib/math/Conversions.java
 * https://github.com/dirtbikerxz/BaseTalonFXSwerve/blob/main/src/main/java/frc/lib/math/Conversions.java
 */

public class Conversions {

    /**
     * Converts raw Falcon counts to degrees
     * 
     * @param counts    Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * Converts raw Neo counts to degrees
     * 
     * @param counts    Neo counts
     * @param gearRatio Gear ration between Neo and mechanism
     * @return Degrees of rotation of mechanism
     */
    public static double neoToDegrees(double counts, double gearRatio) {
        return counts / (gearRatio / 360);
    }

    /**
     * Converts degrees to raw Falcon counts
     * 
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    /**
     * Converts degrees to raw Neo counts
     * 
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Neo and Mechanism
     * @return Neo Counts
     */
    public static double degreesToNeo(double degrees, double gearRatio) {
        return degrees * (gearRatio / 360);
    }

    /**
     * Converts raw Falcon counts to RPM
     * 
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
     *                       Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * Converts RPM to raw Falcon counts
     * 
     * @param RPM       RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon
     *                  RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * Converts raw Falcon counts to meters per second
     * 
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
     *                       Falcon RPM)
     * @return Falcon Velocity in miles per second
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * Converts meters per second to raw Falcon counts
     * 
     * @param velocity      Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Falcon and Mechanism (set to 1 for
     *                      Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * Converts raw Falcon counts to meters
     * 
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference  Circumference of Wheel in meters
     * @return Falcon position in meters
     */
    public static double falconToMeters(double velocityCount, double circumference, double gearRatio) {
        return velocityCount * ((circumference) / (gearRatio * 2048.0));
    }

    /**
     * @param wheelRPS      Wheel Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double RPSToMPS(double wheelRPS, double circumference) {
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param wheelMPS      Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Rotations per Second)
     */
    public static double MPSToRPS(double wheelMPS, double circumference) {
        double wheelRPS = wheelMPS / circumference;
        return wheelRPS;
    }

    /**
     * @param wheelRotations Wheel Position: (in Rotations)
     * @param circumference  Wheel Circumference: (in Meters)
     * @return Wheel Distance: (in Meters)
     */
    public static double rotationsToMeters(double wheelRotations, double circumference) {
        double wheelMeters = wheelRotations * circumference;
        return wheelMeters;
    }

    /**
     * @param wheelMeters   Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    public static double metersToRotations(double wheelMeters, double circumference) {
        double wheelRotations = wheelMeters / circumference;
        return wheelRotations;
    }

}

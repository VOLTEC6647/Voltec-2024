/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.math;

public class Functions {

    /**
     * Clamps a number between to the nearest min/max value
     * 
     * @param num Number to clamp
     * @param min Minimum value
     * @param max Maximum value
     * @return Clamped number
     */
    public static double clamp(double num, double min, double max) {
        return Math.max(min, Math.min(max, num));
    }

    /**
     * Handles controller deathband
     * 
     * @param value    Controller stick value
     * @param deadband Deathband
     * @return Filtered value
     */
    public static double handleDeadband(double value, double deadband) {
        deadband = Math.abs(deadband);
        if (deadband == 1) {
            return 0;
        }
        double scaledValue = (value + (value < 0 ? deadband : -deadband)) / (1 - deadband);

        return (Math.abs(value) > Math.abs(deadband)) ? scaledValue : 0;
    }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 22 01 2024
 */
package com.team6647.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public double topMotorVelocity = 0.0;
        public double bottomMotorVelocity = 0.0;

        public double topMotorPosition = 0.0;
        public double bottomMotorPosition = 0.0;

        public double topMotorCurrent = 0.0;
        public double bottomMotorCurrent = 0.0;
        public double topMotorVoltage = 0.0;
        public double bottomMotorVoltage = 0.0;
        public double topMotorTemperature = 0.0;
        public double bottomMotorTemperature = 0.0;

        public boolean beamBrake = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setShooterVelocity(double velocity) {
    }

    public default void setTopPIDF(double p, double i, double d, double s, double v, double a) {
    }

    public default void setBottomPIDF(double p, double i, double d, double s, double v, double a) {
    }

    public default void runFlywheelCharacterization(double volts){
    }
}

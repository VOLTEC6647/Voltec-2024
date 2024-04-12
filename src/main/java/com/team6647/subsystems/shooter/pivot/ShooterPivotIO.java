/**
 * Written by Juan Pablo Gutiérrez  
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {

    @AutoLog
    public static class ShooterPivotIOInputs {

        public boolean cancoderConnected = true;
        public boolean shooterPivotLeftMotorConnected = true;
        public boolean shooterPivotRightMotorConnected = true;

        public double cancoderAbsolutePosition = 0.0;
        public double cancoderAbsoluteVelocity = 0.0;

        public double shooterPivotLeftMotorPosition = 0.0;
        public double shooterPivotRightMotorPosition = 0.0;
        public double shooterPivotLeftMotorVelocity = 0.0;
        public double shooterPivotRightMotorVelocity = 0.0;

        public double shooterPivotLeftMotorAppliedVolts = 0.0;
        public double shooterPivotRightMotorAppliedVolts = 0.0;
        public double shooterPivotLeftMotorCurrent = 0.0;
        public double shooterPivotRightMotorCurrent = 0.0;

        public double shooterPivotLeftMotorTemperatureCelsius = 0.0;
        public double shooterPivotRightMotorTemperatureCelsius = 0.0;

        public boolean inTolerance = false;
        public double setpoint = 0.0;
        public boolean disabled = false;

        public boolean limitSwitchPressed = false;
    }

    public default void updateInputs(ShooterPivotIOInputs inputs) {
    }

    public default void setShooterReference(double setpoint, boolean homed) {
    }

    public default void setPIDVel(double p, double i, double d, double maxVel, double maxAccel) {
    }

    public default void disablePivot() {
    }

    public default void enablePivot() {

    }

    public default void runPivotVolts(double volts) {
    }
}

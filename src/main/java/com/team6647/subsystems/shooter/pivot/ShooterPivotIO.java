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

        public double cancoderAbsolutePosition = 0.0;
        public double cancoderAbsoluteVelocity = 0.0;

        public double shooterPivotLeftMotorPosition = 0.0;
        public double shooterPivotRightMotorPosition = 0.0;
        public double shooterPivotLeftMotorVelocity = 0.0;
        public double shooterPivotRightMotorVelocity = 0.0;

        public double shooterPivotLeftMotorAppliedVolts = 0.0;
        public double shooterPivotRightMotorAppliedVolts = 0.0;

        public double shooterPivotLeftMotorTemperatureCelsius = 0.0;
        public double shooterPivotRightMotorTemperatureCelsius = 0.0;

        public double arbitraryFeedforward = 0.0;
        public double pidValue = 0.0;
        public double output = 0.0;
        public boolean inTolerance = false;
        public double setpoint;

        public boolean limitSwitchPressed = false;
    }

    public default void updateInputs(ShooterPivotIOInputs inputs) {
    }

    public default void setShooterReference(double setpoint) {
    }

    public default void setPIDF(double p, double i, double d, double f) {
    }

    public default void disablePivot() {
    }

    public default void runPivotCharacterization(double volts) {
    }
}

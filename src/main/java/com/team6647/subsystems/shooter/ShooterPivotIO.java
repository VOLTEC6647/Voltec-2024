/**
 * Written by Juan Pablo Gutiérrez  
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {

    @AutoLog
    public static class ShooterPivotIOInputs {
        public double shooterAbsoluteEncoderPosition = 0.0;
        public double shooterAbsoluteEncoderVelocity = 0.0;

        public double shooterPivotAppliedVolts = 0.0;

        public double pivotMotorPosition = 0.0;

        public boolean inTolerance = false;
    }

    public default void updateInputs(ShooterPivotIOInputs inputs) {
    }

    public default void setShooterReference(double setpoint) {
    }

    public default void setPIDF(double p, double i, double d, double f) {
    }
}

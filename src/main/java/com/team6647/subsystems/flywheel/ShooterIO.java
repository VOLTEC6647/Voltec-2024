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

        public boolean beamBrake = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setShooterVelocity(double velocity) {
    }

    public default void setPIDF(double p, double i, double d, double f) {
    }
}

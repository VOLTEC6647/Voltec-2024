/**
 * Writen by Juan Pablo Gutiérrez
 * 
 * 24 01 2024
 */

package com.team6647.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeMotorVelocity = 0.0;
        public double intakeMotorAppliedVoltage = 0.0;
        public double intakeMotorCurrent = 0.0;
        public boolean intakeBeamBrake = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setIntakeVelocity(double velocity) {
    }
}

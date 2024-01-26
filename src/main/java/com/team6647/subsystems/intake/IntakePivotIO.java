
/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 26 01 2024
 
 */

package com.team6647.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {

    @AutoLog
    public static class IntakePivoIOInputs {
        public double intakePivotMotorVelocity = 0.0;
        public double intakePivotMotorAppliedVoltage = 0.0;
        public double intakePivotMotorPosition = 0.0;
        public double intakePivotAbsoluteEncoderPosition = 0.0;
    }

    public default void updateInputs(IntakePivoIOInputs inputs) {
    }

    public default void setIntakeVoltage(double velocity) {
    }
}

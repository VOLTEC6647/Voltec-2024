
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
        public double intakePivotLeftMotorVelocity = 0.0;
        public double intakePivotLeftMotorAppliedVoltage = 0.0;
        public double intakePivotLeftMotorPosition = 0.0;
        public double intakePivotLeftMotorCurrent = 0.0;

        public double intakePivotRightMotorVelocity = 0.0;
        public double intakePivotRightMotorAppliedVoltage = 0.0;
        public double intakePivotRightMotorPosition = 0.0;
        public double intakePivtoRightMotorCurrent = 0.0;

        public double intakePivotAbsoluteEncoderPosition = 0.0;

        public boolean intakeBeamBrake = false;
    }

    public default void updateInputs(IntakePivoIOInputs inputs) {
    }

    public default void setIntakeVoltage(double leftMotorVolts, double rightMotorVolts) {
    }
}

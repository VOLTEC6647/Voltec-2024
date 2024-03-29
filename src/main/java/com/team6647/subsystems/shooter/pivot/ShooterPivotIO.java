/**
 * Written by Juan Pablo Gutiérrez  
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter.pivot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface ShooterPivotIO {

    @AutoLog
    public static class ShooterPivotIOInputs {

        public MutableMeasure<Angle> cancoderAbsolutePosition = MutableMeasure.zero(Rotations);
        public MutableMeasure<Velocity<Angle>> cancoderAbsoluteVelocity = MutableMeasure.zero(RotationsPerSecond);

        public MutableMeasure<Angle> shooterPivotLeftMotorPosition = MutableMeasure.zero(Rotations);
        public MutableMeasure<Angle> shooterPivotRightMotorPosition = MutableMeasure.zero(Rotations);
        public MutableMeasure<Velocity<Angle>> shooterPivotLeftMotorVelocity = MutableMeasure.zero(RotationsPerSecond);
        public MutableMeasure<Velocity<Angle>> shooterPivotRightMotorVelocity = MutableMeasure.zero(RotationsPerSecond);

        public MutableMeasure<Voltage> shooterPivotLeftMotorAppliedVolts = MutableMeasure.zero(Volts);
        public MutableMeasure<Voltage> shooterPivotRightMotorAppliedVolts = MutableMeasure.zero(Volts);

        public boolean inTolerance = false;

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
}

package com.andromedalib.andromedaSwerve.andromedaModule;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface AndromedaModuleIO {

    @AutoLog
    public static class AndromedaModuleIOInputs {
        public boolean angleMotorConnected = true;
        public boolean driveMotorConnected = true;

        public MutableMeasure<Velocity<Distance>> driveVelocity = MutableMeasure.zero(MetersPerSecond);
        public MutableMeasure<Distance> drivePosition = MutableMeasure.zero(Meters);
        public MutableMeasure<Voltage> driveApplied = MutableMeasure.zero(Volts);
        public MutableMeasure<Velocity<Velocity<Distance>>> driveAcceleration = MutableMeasure
                .zero(MetersPerSecondPerSecond);

        public MutableMeasure<Angle> steerAngle = MutableMeasure.zero(Rotation);
        public MutableMeasure<Voltage> turnAppliedVolts = MutableMeasure.zero(Volts);
        public MutableMeasure<Velocity<Angle>> turnVelocity = MutableMeasure.zero(RotationsPerSecond);
        public MutableMeasure<Angle> encoderAbsolutePosition = MutableMeasure.zero(Rotations);

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositions = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(AndromedaModuleIOInputs inputs) {
    }

    /** Sets the turn motor to the position */
    public default void setTurnPosition(Rotation2d angle) {
    }

    /** Sets the drive motor to a voltage */
    public default void setDriveVelocity(double velocity) {
    }

    /** Runs the drive motor for characterization */
    public default void runDriveCharacterization(Measure<Voltage> volts) {
    }

}

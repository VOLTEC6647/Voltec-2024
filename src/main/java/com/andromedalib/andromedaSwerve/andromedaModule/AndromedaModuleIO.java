package com.andromedalib.andromedaSwerve.andromedaModule;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public interface AndromedaModuleIO {

    @AutoLog
    public static class AndromedaModuleIOInputs {
        public boolean angleMotorConnected = true;
        public boolean driveMotorConnected = true;
        public boolean cancoderConnected = true;

        public double driveVelocity = 0.0;
        public double drivePosition = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveAcceleration = 0.0;
        public double driveCurrent = 0.0;

        public Rotation2d steerAngle = new Rotation2d();
        public double turnAppliedVolts = 0.0;
        public double turnCurrent = 0.0;
        public double turnVelocity = 0.0;
        public Rotation2d encoderAbsolutePosition = new Rotation2d();

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

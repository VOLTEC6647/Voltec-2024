package com.andromedalib.andromedaSwerve.andromedaModule;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AndromedaModuleIO {

    @AutoLog
    public static class AndromedaModuleIOInputs {
        public double driveVelocity = 0.0;
        public double drivePosition = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveAcceleration = 0.0;

        public Rotation2d steerAngle = new Rotation2d(0.0);
        public double turnAppliedVolts = 0.0;
        public double turnVelocity = 0.0;
        public Rotation2d encoderAbsolutePosition = new Rotation2d(0.0);
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
    public default void runDriveCharacterization(double volts) {
    }

}

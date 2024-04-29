/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.andromedaSwerve.andromedaModule;

import org.littletonrobotics.junction.Logger;

import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig;
import com.andromedalib.util.Alert;
import com.andromedalib.util.Alert.AlertType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import lombok.Getter;

public class AndromedaModule {
    @Getter
    private final int moduleNumber;
    private final String moduleName;

    private AndromedaModuleIO io;
    private AndromedaModuleIOInputsAutoLogged inputs = new AndromedaModuleIOInputsAutoLogged();

    private Rotation2d lastAngle;
    private AndromedaSwerveConfig andromedaSwerveConfig;

    private final Alert driveMotorDisconnectedAlert;
    private final Alert turnMotorDisconnectedalert;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
    public static final double ODOMETRY_FREQUENCY = 250.0;

    public AndromedaModule(int moduleNumber, String name,
            AndromedaSwerveConfig swerveConfig, AndromedaModuleIO io) {
        this.io = io;
        this.moduleName = name;
        this.moduleNumber = moduleNumber;
        this.andromedaSwerveConfig = swerveConfig;

        driveMotorDisconnectedAlert = new Alert("Module [" + moduleName + "] Drive Motor disconnected",
                AlertType.WARNING);
        turnMotorDisconnectedalert = new Alert("Module [" + moduleName + "] Angle Motor disconnected",
                AlertType.WARNING);

        lastAngle = getAngle();
    }

    public void periodic() {
        Logger.processInputs("Swerve/" + moduleName, inputs);

        driveMotorDisconnectedAlert.set(!inputs.driveMotorConnected);
        turnMotorDisconnectedalert.set(!inputs.angleMotorConnected);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositions[i];
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    /**
     * Update inputs without running the rest of the periodic logic. This is useful
     * since these
     * updates need to be properly thread-locked.
     */
    public void updateInputs() {
        io.updateInputs(inputs);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());

        setAngle(desiredState);
        setSpeed(desiredState);
    }

    /**
     * Sets the turning motor´+´+ angle to its desired state
     * 
     * @param desiredState {@link SwerveModuleState} to apply
     */
    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (andromedaSwerveConfig.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        io.setTurnPosition(angle);
        lastAngle = angle;
    }

    /**
     * Sets the drive motor speed to its desired state
     * 
     * @param desiredState {@link SwerveModuleState} to apply
     * @param isOpenLoop   True if open loop feedback is enabled
     */
    private void setSpeed(SwerveModuleState desiredState) {
        io.setDriveVelocity(desiredState.speedMetersPerSecond);
    }

    /**
     * Gets the current position and angle as a {@link SwerveModuleState}
     * 
     * @return
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveSpeed(), getAngle());
    }

    /**
     * Gets the current speed of the drive motor
     * 
     * @return Speed in meters per second
     */
    public double getDriveSpeed() {
        return inputs.driveVelocity;
    }

    /**
     * Gets the current angle of the turning motor
     * 
     * @return Angle in {@link Rotation2d}
     */
    private Rotation2d getAngle() {
        return inputs.steerAngle;
    }

    /**
     * Gets the current voltage of the drive motor
     * 
     * @return Voltage in volts
     */
    public double getDriveVoltage() {
        return inputs.driveAppliedVolts;
    }

    /**
     * Gets the current position of the module
     * 
     * @return Current position in {@link SwerveModulePosition}
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePosition, getAngle());
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /**
     * Runs the drive motor for characterization
     * 
     * @param volts Voltage to run the motor at
     */
    public void runCharacterization(Measure<Voltage> volts) {
        io.setTurnPosition(new Rotation2d());
        io.runDriveCharacterization(volts);
    }
}

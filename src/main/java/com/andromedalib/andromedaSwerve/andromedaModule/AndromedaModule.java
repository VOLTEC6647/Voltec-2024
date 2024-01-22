/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.andromedaSwerve.andromedaModule;

import org.littletonrobotics.junction.Logger;

import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class AndromedaModule {
    private final int moduleNumber;
    private final String moduleName;

    private AndromedaModuleIO io;
    private AndromedaModuleIOInputsAutoLogged inputs = new AndromedaModuleIOInputsAutoLogged();

    private Rotation2d lastAngle;
    private AndromedaSwerveConfig andromedaSwerveConfig;

    public AndromedaModule(int moduleNumber, String name,
            AndromedaSwerveConfig swerveConfig, AndromedaModuleIO io) {
        this.io = io;
        this.moduleName = name;
        this.moduleNumber = moduleNumber;
        this.andromedaSwerveConfig = swerveConfig;

        lastAngle = getAngle();
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/" + moduleName, inputs);
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

    public double getDriveSpeed() {
        return inputs.driveVelocity;
    }

    private Rotation2d getAngle() {
        return inputs.steerAngle;
    }

    public int getModuleNumber() {
        return moduleNumber;
    }

    public double getDriveVoltage() {
        return inputs.driveAppliedVolts;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePosition, getAngle());
    }

    public void runCharacterization(double volts) {
        io.setTurnPosition(new Rotation2d());
        io.runDriveCharacterization(volts);
    }
}

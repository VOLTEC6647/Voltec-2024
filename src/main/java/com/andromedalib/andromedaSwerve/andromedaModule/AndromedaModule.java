/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.andromedaSwerve.andromedaModule;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig;
import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig.Mode;
import com.andromedalib.math.Conversions;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController driveFeedback;
    private final PIDController turnFeedback;

    @AutoLogOutput
    private double pidVal = 0.0;

    public AndromedaModule(int moduleNumber, String name,
            AndromedaSwerveConfig swerveConfig, Mode mode, AndromedaModuleIO io) {
        this.io = io;
        this.moduleName = name;
        this.moduleNumber = moduleNumber;
        this.andromedaSwerveConfig = swerveConfig;

        /* Remove unused warning */

        lastAngle = getAngle();

        switch (mode) {
            case REAL:
            case REPLAY:
                driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
                driveFeedback = new PIDController(0.1, 0.0, 0.0);
                turnFeedback = new PIDController(3.5, 0.0, 0.0);
                break;
            case SIM:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
                driveFeedback = new PIDController(0.1, 0.0, 0.0);
                turnFeedback = new PIDController(10.0, 0.0, 0.0);
                break;
            default:
                driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
                driveFeedback = new PIDController(0.0, 0.0, 0.0);
                turnFeedback = new PIDController(0.0, 0.0, 0.0);
                break;
        }

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/" + Integer.toString(moduleNumber) + "-" + moduleName, inputs);

        SignalLogger.writeDouble(moduleNumber + "-drive-volts", getDriveVoltage());
        SignalLogger.writeDouble(moduleNumber + "-drive-position", getDrivePosition());
        SignalLogger.writeDouble(moduleNumber + "-drive-velocity", getDriveSpeed());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean internalControl) {
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());

        setAngle(desiredState, internalControl);
        setSpeed(desiredState, internalControl);
    }

    /**
     * Sets the turning motor´+´+ angle to its desired state
     * 
     * @param desiredState {@link SwerveModuleState} to apply
     */
    private void setAngle(SwerveModuleState desiredState, boolean internalControl) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (andromedaSwerveConfig.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        if (internalControl) {
            io.setTurnVoltage(angle.getRotations(), internalControl);
        } else {
            io.setTurnVoltage(turnFeedback.calculate(getAngle().getRotations(),
                    angle.getRotations()), internalControl);
        }

        lastAngle = angle;
    }

    /**
     * Sets the drive motor speed to its desired state
     * 
     * @param desiredState {@link SwerveModuleState} to apply
     * @param isOpenLoop   True if open loop feedback is enabled
     */
    private void setSpeed(SwerveModuleState desiredState, boolean internaControl) {
        if (internaControl) {
            VelocityVoltage velocity = new VelocityVoltage(0);
            velocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
                    andromedaSwerveConfig.wheelCircumference);
            velocity.FeedForward = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
            io.setDriveVoltage(velocity.Velocity, internaControl);
        } else {
            double adjustedSpeed = desiredState.speedMetersPerSecond / 0.051; // TODO FIX
            pidVal = driveFeedback.calculate(inputs.driveVelocity, adjustedSpeed);
            io.setDriveVoltage(pidVal, internaControl);
        }
    }

    /* Telemetry */

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

    private double getDrivePosition() {
        return inputs.drivePosition;
    }

    public int getModuleNumber() {
        return moduleNumber;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePosition, getAngle());
    }

    public double getDriveVoltage() {
        return inputs.driveAppliedVolts;
    }

    public double getTurnVoltage() {
        return inputs.driveAppliedVolts;
    }

    /**
     * Runs the module with the specified voltage while controlling to zero degrees.
     */
    /*
     * public void runCharacterization(double volts) {
     * // Closed loop turn control
     * setAngle(new SwerveModuleState(0, new Rotation2d()), true);
     * 
     * // Open loop drive control
     * io.setDriveVoltage(volts, false);
     * }
     */
}

/**<
 * Written by Juan Pablo Gutierrez
 */

package com.andromedalib.andromedaSwerve.config;

import com.andromedalib.andromedaSwerve.utils.AndromedModuleIDs;
import com.andromedalib.motorControllers.SuperSparkMaxConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.util.Units;

public class AndromedaModuleConfig {
    public final AndromedModuleIDs moduleIDs;

    public final TalonFXConfiguration driveMotorConfiguration;
    public final TalonFXConfiguration turningMotorConfiguration;
    public final CANcoderConfiguration cancoderConfiguration;

    public final SuperSparkMaxConfiguration driveMotorSparkMaxConfiguration;
    public final SuperSparkMaxConfiguration turningMotorSparkMaxConfiguration;

    public final double wheelDiameter;
    public double wheelCircumference;
    public final String swerveCANBus;

    public final ModuleMotorConfig motorConfig;

    public AndromedaModuleConfig(AndromedModuleIDs moduleIDs, TalonFXConfiguration driveMotorConfiguration,
            TalonFXConfiguration turningMotorConfiguration, CANcoderConfiguration encoderMotorConfiguration,
            double wheelDiameter, String swerveCANBus, ModuleMotorConfig motorConfig) {
        this.moduleIDs = moduleIDs;
        this.driveMotorConfiguration = driveMotorConfiguration;
        this.turningMotorConfiguration = turningMotorConfiguration;
        this.cancoderConfiguration = encoderMotorConfiguration;
        this.wheelDiameter = wheelDiameter;
        this.swerveCANBus = swerveCANBus;
        this.wheelCircumference = Units.inchesToMeters(wheelDiameter) * Math.PI;

        this.driveMotorSparkMaxConfiguration = null;
        this.turningMotorSparkMaxConfiguration = null;
        this.motorConfig = motorConfig;

    }

    public AndromedaModuleConfig(AndromedModuleIDs moduleIDs,
            SuperSparkMaxConfiguration driveMotorSparkMaxConfiguration,
            SuperSparkMaxConfiguration turningMotorSparkMaxConfiguration, CANcoderConfiguration eNcoderConfiguration,
            double wheelDiameter, ModuleMotorConfig motorConfig) {
        this.moduleIDs = moduleIDs;
        this.driveMotorSparkMaxConfiguration = driveMotorSparkMaxConfiguration;
        this.turningMotorSparkMaxConfiguration = turningMotorSparkMaxConfiguration;
        this.cancoderConfiguration = eNcoderConfiguration;
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = Units.inchesToMeters(wheelDiameter) * Math.PI;

        this.swerveCANBus = "";
        this.driveMotorConfiguration = null;
        this.turningMotorConfiguration = null;
        this.motorConfig = motorConfig;
    }

    public enum ModuleMotorConfig {
        FALCON_CONFIG,
        SPARKMAX_CONFIG
    }
}

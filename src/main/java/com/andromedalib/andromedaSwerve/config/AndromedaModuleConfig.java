/**<
 * Written by Juan Pablo Gutierrez
 */

package com.andromedalib.andromedaSwerve.config;

import com.andromedalib.andromedaSwerve.utils.AndromedModuleIDs;
import com.andromedalib.motorControllers.SuperSparkMaxConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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

    public static enum AndromedaProfiles {
        ANDROMEDA_CONFIG,
        NEO_CONFIG
    }

    /**
     * Gets the {@AndromedaProfileConfig} based on {@link} AndromedaProfiles}
     * selection
     * 
     * @param profile Profiles selection
     * @return {@link AndromedaProfileConfig} selection
     */
    public static AndromedaModuleConfig getConfig(AndromedaProfiles profile, AndromedModuleIDs moduleIDs) {
        switch (profile) {
            case ANDROMEDA_CONFIG:
                return andromedModuleConfig(moduleIDs);
            case NEO_CONFIG:
                return neoAndromedModuleConfig(moduleIDs);
        }
        return andromedModuleConfig(moduleIDs);
    }

    private static AndromedaModuleConfig andromedModuleConfig(AndromedModuleIDs moduleIDs) {
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration turningMotorConfig = new TalonFXConfiguration();
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

        String swerveCANBus = "6647_CANivore";

        double wheelDiameter = Units.inchesToMeters(4.0);

        double steeringGearRatio = ((150.0 / 7.0) / 1.0);
        double driveGearRatio = (6.75 / 1.0);

        double turningKp = 9.9;
        double turningKi = 0.0;
        double turningKd = 0.089;

        double driveKp = 0.1;
        double driveKi = 0.0;
        double driveKd = 0.0;
        double driveKs = 0.18225; // TODO TUNE
        double driveKv = 2.3503; // TODO TUNE
        double driveKa = 0.05414; // TODO TUNE

        double openLoopRamp = 0.25;
        double closedLoopRamp = 0.0;

        InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive; // False
        InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive; // True
        SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive; // False

        /* Current Limiting */
        int driveContinuousCurrentLimit = 35;
        int drivePeakCurrentLimit = 60;
        double drivePeakCurrentDuration = 0.1;
        boolean driveEnableCurrentLimit = true;

        int angleContinuousCurrentLimit = 25;
        int anglePeakCurrentLimit = 40;
        double anglePeakCurrentDuration = 0.1;
        boolean angleEnableCurrentLimit = true;

        /* Drive Motor Configuration */
        driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
        driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;

        driveMotorConfig.Slot0.kP = driveKp;
        driveMotorConfig.Slot0.kI = driveKi;
        driveMotorConfig.Slot0.kD = driveKd;
        driveMotorConfig.Slot0.kS = driveKs;
        driveMotorConfig.Slot0.kV = driveKv;
        driveMotorConfig.Slot0.kA = driveKa;

        driveMotorConfig.MotorOutput.Inverted = driveMotorInvert;

        driveMotorConfig.Feedback.SensorToMechanismRatio = driveGearRatio;
        driveMotorConfig.Audio.BeepOnBoot = true;
        driveMotorConfig.Audio.BeepOnConfig = true;

        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = driveContinuousCurrentLimit;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = driveEnableCurrentLimit;
        driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = drivePeakCurrentLimit;
        driveMotorConfig.CurrentLimits.SupplyTimeThreshold = drivePeakCurrentDuration;

        ModuleMotorConfig motorConfig = ModuleMotorConfig.FALCON_CONFIG;
        /* CANCoder */

        cancoderConfig.MagnetSensor.SensorDirection = canCoderInvert;

        /* Turning Motor */

        turningMotorConfig.Slot0.kP = turningKp;
        turningMotorConfig.Slot0.kI = turningKi;
        turningMotorConfig.Slot0.kD = turningKd;

        turningMotorConfig.MotorOutput.Inverted = angleMotorInvert;

        turningMotorConfig.Feedback.SensorToMechanismRatio = steeringGearRatio;
        turningMotorConfig.Audio.BeepOnBoot = true;
        turningMotorConfig.Audio.BeepOnConfig = true;

        turningMotorConfig.CurrentLimits.SupplyCurrentLimit = angleContinuousCurrentLimit;
        turningMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = angleEnableCurrentLimit;
        turningMotorConfig.CurrentLimits.SupplyCurrentThreshold = anglePeakCurrentLimit;
        turningMotorConfig.CurrentLimits.SupplyTimeThreshold = anglePeakCurrentDuration;

        turningMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;


        return new AndromedaModuleConfig(moduleIDs, driveMotorConfig, turningMotorConfig, cancoderConfig, wheelDiameter,
                swerveCANBus, motorConfig);
    }

    private static AndromedaModuleConfig neoAndromedModuleConfig(AndromedModuleIDs moduleIDs) {
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

        double wheelDiameter = Units.inchesToMeters(4.0);

        double steeringGearRatio = ((150.0 / 7.0) / 1.0);
        double driveGearRatio = (6.75 / 1.0);

        double turningKp = 0.007;
        double turningKi = 0.0;
        double turningKd = 0.0;
        double turningKf = 0.0;

        double driveKp = 0.3;
        double driveKi = 0.0;
        double driveKd = 0.0;
        double driveKf = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = true;
        SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive; // False

        /* Current Limiting */
        int driveContinuousCurrentLimit = 35;

        int angleContinuousCurrentLimit = 25;

        ModuleMotorConfig motorConfig = ModuleMotorConfig.SPARKMAX_CONFIG;

        /* CANCoder */
        cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cancoderConfig.MagnetSensor.SensorDirection = canCoderInvert;
        cancoderConfig.MagnetSensor.MagnetOffset = moduleIDs.angleOffset.getDegrees();

        SuperSparkMaxConfiguration driveMotorConfig = new SuperSparkMaxConfiguration(driveGearRatio, driveKp, driveKi,
                driveKd, driveKf, driveMotorInvert, driveContinuousCurrentLimit);

        SuperSparkMaxConfiguration turningMotorConfig = new SuperSparkMaxConfiguration(steeringGearRatio, turningKp,
                turningKi, turningKd, turningKf, angleMotorInvert, angleContinuousCurrentLimit);

        return new AndromedaModuleConfig(moduleIDs, driveMotorConfig, turningMotorConfig, cancoderConfig,
                wheelDiameter, motorConfig);

    }
}

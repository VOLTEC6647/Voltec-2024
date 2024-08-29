package com.team6647.subsystems.shooter.pivot;

import com.andromedalib.sensors.SuperCANCoder;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.andromedalib.motorControllers.SuperTalonFX;
import com.team6647.util.Constants.RobotConstants;
import com.team6647.util.Constants.ShooterConstants;

public class ShooterPivotIOTalonFX implements ShooterPivotIO {
        /*

        private static SuperTalonFX shooterPivotLeftMotor;
        private static SuperTalonFX shooterPivotRightMotor;

        private static SuperCANCoder shooterPivotEncoder;
        private static MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

        private final StatusSignal<Double> shooterPivotLeftMotorPosition;
        private final StatusSignal<Double> shooterPivotRightMotorPosition;
        private final StatusSignal<Double> shooterPivotLeftMotorVelocity;
        private final StatusSignal<Double> shooterPivotRightMotorVelocity;
        private final StatusSignal<Double> shooterPivotLeftMotorAppliedVolts;
        private final StatusSignal<Double> shooterPivotRightMotorAppliedVolts;
        private final StatusSignal<Double> cancoderAbsolutePosition;
        private final StatusSignal<Double> cancoderAbsoluteVelocity;
        private final StatusSignal<Double> shooterPivotLeftMotorTemperature;
        private final StatusSignal<Double> shooterPivotRightMotorTemperature;
        private final StatusSignal<Double> shooterPivotRightMotorCurrent;
        private final StatusSignal<Double> shooterPivotLeftMotorCurrent;

        private double setpoint;
        private boolean disabled;

        public ShooterPivotIOTalonFX() {
                CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
                cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
                cancoderConfig.MagnetSensor.SensorDirection = ShooterConstants.shooterPivotEncoderInverted;
                cancoderConfig.MagnetSensor.MagnetOffset = ShooterConstants.shooterPivotEncoderOffset;
                shooterPivotEncoder = new SuperCANCoder(ShooterConstants.shooterPivotCANCoderID, cancoderConfig,
                                RobotConstants.mechanismsCANnivore);

                // Config left motor

                TalonFXConfiguration leftConfig = new TalonFXConfiguration();
                leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                leftConfig.Audio.BeepOnConfig = true;
                leftConfig.Feedback.FeedbackRemoteSensorID = shooterPivotEncoder.getDeviceID();
                leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                leftConfig.Feedback.RotorToSensorRatio = 93.75;
                leftConfig.Slot0.kP = ShooterConstants.pivotKp;
                leftConfig.Slot0.kI = ShooterConstants.pivotKi;
                leftConfig.Slot0.kD = ShooterConstants.pivotKd;
                leftConfig.Slot0.kS = 0.25546;
                leftConfig.Slot0.kA = 0.1;
                leftConfig.Slot0.kV = 0.082361;
                leftConfig.Slot0.kG = 0.18398;
                leftConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

                leftConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.pivotMaxAccel;
                leftConfig.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.pivotMaxVel;

                shooterPivotLeftMotor = new SuperTalonFX(
                                ShooterConstants.shooterPivotLeftMotorID,
                                leftConfig, RobotConstants.mechanismsCANnivore);

                                
                // Config right motor

                TalonFXConfiguration rightConfig = new TalonFXConfiguration();
                rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                rightConfig.Audio.BeepOnConfig = true;

                shooterPivotRightMotor = new SuperTalonFX(
                                ShooterConstants.shooterPivotRightMotorID,
                                rightConfig, RobotConstants.mechanismsCANnivore);

                shooterPivotRightMotor.setControl(new Follower(ShooterConstants.shooterPivotLeftMotorID, true));

                // Config status signals

                shooterPivotLeftMotorPosition = shooterPivotLeftMotor.getPosition();
                shooterPivotRightMotorPosition = shooterPivotRightMotor.getPosition();
                shooterPivotLeftMotorVelocity = shooterPivotLeftMotor.getVelocity();
                shooterPivotRightMotorVelocity = shooterPivotRightMotor.getVelocity();
                shooterPivotLeftMotorAppliedVolts = shooterPivotLeftMotor.getMotorVoltage();
                shooterPivotRightMotorAppliedVolts = shooterPivotRightMotor.getMotorVoltage();
                cancoderAbsolutePosition = shooterPivotEncoder.getAbsolutePosition();
                cancoderAbsoluteVelocity = shooterPivotEncoder.getVelocity();
                shooterPivotLeftMotorTemperature = shooterPivotLeftMotor.getDeviceTemp();
                shooterPivotRightMotorTemperature = shooterPivotRightMotor.getDeviceTemp();
                shooterPivotLeftMotorCurrent = shooterPivotLeftMotor.getSupplyCurrent();
                shooterPivotRightMotorCurrent = shooterPivotRightMotor.getSupplyCurrent();

                BaseStatusSignal.setUpdateFrequencyForAll(50.0,
                                shooterPivotLeftMotorPosition,
                                shooterPivotRightMotorPosition,
                                shooterPivotLeftMotorVelocity,
                                shooterPivotRightMotorVelocity,
                                shooterPivotLeftMotorAppliedVolts,
                                shooterPivotRightMotorAppliedVolts,
                                cancoderAbsolutePosition,
                                cancoderAbsoluteVelocity,
                                shooterPivotLeftMotorTemperature,
                                shooterPivotRightMotorTemperature,
                                shooterPivotLeftMotorCurrent,
                                shooterPivotRightMotorCurrent);

        }

        @Override
        public void updateInputs(ShooterPivotIOInputs inputs) {
                inputs.cancoderConnected = BaseStatusSignal.refreshAll(
                                cancoderAbsolutePosition, cancoderAbsoluteVelocity).isOK();

                inputs.shooterPivotLeftMotorConnected = BaseStatusSignal.refreshAll(
                                shooterPivotLeftMotorPosition,
                                shooterPivotLeftMotorVelocity,
                                shooterPivotLeftMotorAppliedVolts,
                                shooterPivotLeftMotorTemperature,
                                shooterPivotLeftMotorCurrent).isOK();

                inputs.shooterPivotRightMotorConnected = BaseStatusSignal.refreshAll(
                                shooterPivotRightMotorPosition,
                                shooterPivotRightMotorVelocity,
                                shooterPivotRightMotorAppliedVolts,
                                shooterPivotRightMotorTemperature,
                                shooterPivotRightMotorCurrent).isOK();

                inputs.cancoderAbsolutePosition = cancoderAbsolutePosition.getValueAsDouble() * 360;

                inputs.cancoderAbsoluteVelocity = cancoderAbsoluteVelocity.getValueAsDouble();

                inputs.shooterPivotLeftMotorPosition = shooterPivotLeftMotorPosition.getValueAsDouble() * 360;

                inputs.shooterPivotRightMotorPosition = shooterPivotRightMotorPosition.getValueAsDouble();

                inputs.shooterPivotLeftMotorVelocity = shooterPivotLeftMotorVelocity.getValueAsDouble();

                inputs.shooterPivotRightMotorVelocity = shooterPivotRightMotorVelocity.getValueAsDouble();

                inputs.shooterPivotLeftMotorAppliedVolts = shooterPivotLeftMotorAppliedVolts.getValueAsDouble();

                inputs.shooterPivotRightMotorAppliedVolts = shooterPivotRightMotorAppliedVolts.getValueAsDouble();

                inputs.shooterPivotLeftMotorTemperatureCelsius = shooterPivotLeftMotorTemperature.getValueAsDouble();

                inputs.shooterPivotRightMotorTemperatureCelsius = shooterPivotRightMotorTemperature.getValueAsDouble();

                inputs.shooterPivotLeftMotorCurrent = shooterPivotLeftMotorCurrent.getValueAsDouble();

                inputs.shooterPivotRightMotorCurrent = shooterPivotRightMotorCurrent.getValueAsDouble();

                inputs.inTolerance = Math.abs(cancoderAbsolutePosition.getValueAsDouble() * 360
                                - setpoint * 360) < ShooterConstants.positionTolerance;

                // Sets pivot position based on setpoint

                inputs.setpoint = setpoint * 360;

                inputs.disabled = disabled;
                
                if (!disabled) {
                        shooterPivotLeftMotor.setControl(
                                        motionMagicVoltage.withPosition(setpoint).withEnableFOC(true));
                }
                
        }

        @Override
        public void setShooterReference(double setpoint) {
                this.setpoint = setpoint / 360;
        }

        @Override
        public void disablePivot() {
                disabled = true;

                shooterPivotLeftMotor.disable();
                shooterPivotRightMotor.disable();
                shooterPivotLeftMotor.stopMotor();
                shooterPivotRightMotor.stopMotor();

        }

        @Override
        public void enablePivot() {
                disabled = false;
        }

        @Override
        public void setPIDVel(double p, double i, double d, double maxVel, double maxAccel) {
                TalonFXConfiguration talonConfig = new TalonFXConfiguration();
                shooterPivotLeftMotor.getConfigurator().apply(new TalonFXConfiguration());
                talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                talonConfig.Audio.BeepOnConfig = true;
                talonConfig.Feedback.FeedbackRemoteSensorID = shooterPivotEncoder.getDeviceID();
                talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                talonConfig.Feedback.RotorToSensorRatio = 93.75;
                talonConfig.Slot0.kP = p;
                talonConfig.Slot0.kI = i;
                talonConfig.Slot0.kD = d;
                talonConfig.Slot0.kS = 0.25546;
                talonConfig.Slot0.kA = 0.1;
                talonConfig.Slot0.kV = 0.082361;
                talonConfig.Slot0.kG = 0.18398;
                talonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

                talonConfig.MotionMagic.MotionMagicAcceleration = maxAccel;
                talonConfig.MotionMagic.MotionMagicCruiseVelocity = maxVel;

                shooterPivotLeftMotor.getConfigurator().apply(talonConfig);
        }

        @Override
        public void runPivotVolts(double volts) {
                 shooterPivotLeftMotor.setVoltage(volts);
                 shooterPivotRightMotor.setVoltage(volts);

        }
        */
}

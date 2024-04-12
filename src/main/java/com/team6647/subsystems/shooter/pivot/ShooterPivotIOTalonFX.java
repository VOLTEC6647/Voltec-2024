package com.team6647.subsystems.shooter.pivot;

import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.andromedalib.sensors.SuperCANCoder;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.AutoLogOutput;

import com.andromedalib.motorControllers.SuperTalonFX;
import com.team6647.util.Constants.ShooterConstants;

public class ShooterPivotIOTalonFX implements ShooterPivotIO {

        private static SuperTalonFX shooterPivotLeftMotor = new SuperTalonFX(
                        ShooterConstants.shooterPivotLeftMotorID,
                        GlobalIdleMode.Brake, "6647_Mechanisms");

        private static SuperTalonFX shooterPivotRightMotor = new SuperTalonFX(
                        ShooterConstants.shooterPivotRightMotorID,
                        GlobalIdleMode.Brake, "6647_Mechanisms");

        private static SuperCANCoder shooterPivotEncoder;
        // private static MotionMagicVoltage motionMagicVoltage = new
        // MotionMagicVoltage(0.0).withSlot(0);
        private static DynamicMotionMagicVoltage dynamicMotionMagicVoltage = new DynamicMotionMagicVoltage(0,
                        ShooterConstants.pivotMaxVel, ShooterConstants.pivotMaxAccel, 0).withSlot(0);

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

        @AutoLogOutput(key = "Shooter/homed")
        private boolean homed;

        public ShooterPivotIOTalonFX() {
                CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
                cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
                cancoderConfig.MagnetSensor.SensorDirection = ShooterConstants.shooterPivotEncoderInverted;
                cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
                cancoderConfig.MagnetSensor.MagnetOffset = ShooterConstants.shooterPivotEncoderOffset;
                shooterPivotEncoder = new SuperCANCoder(ShooterConstants.shooterPivotCANCoderID, cancoderConfig,
                                "6647_Mechanisms");

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

                TalonFXConfiguration talonConfig = new TalonFXConfiguration();
                shooterPivotRightMotor.getConfigurator().apply(new TalonFXConfiguration());
                talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                talonConfig.Audio.BeepOnConfig = true;
                talonConfig.Feedback.FeedbackRemoteSensorID = shooterPivotEncoder.getDeviceID();
                talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                talonConfig.Feedback.SensorToMechanismRatio = 1.0;
                talonConfig.Feedback.RotorToSensorRatio = 90;

                shooterPivotRightMotor.getConfigurator().apply(talonConfig);

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

                shooterPivotRightMotor.setControl(new Follower(ShooterConstants.shooterPivotLeftMotorID, true));

                setPIDVel(ShooterConstants.pivotKp, ShooterConstants.pivotKi, ShooterConstants.pivotKd,
                                ShooterConstants.pivotMaxVel, ShooterConstants.pivotMaxAccel);
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

                /* Sets pivot position based on setpoint */

                inputs.setpoint = setpoint * 360;

                inputs.disabled = disabled;

                if (!disabled) {
                        if (homed) {
                                dynamicMotionMagicVoltage.Acceleration = ShooterConstants.pivotHomedMaxAccel;
                                dynamicMotionMagicVoltage.Velocity = ShooterConstants.pivotHomedMaxVel;
                        } else {
                                dynamicMotionMagicVoltage.Acceleration = ShooterConstants.pivotMaxAccel;
                                dynamicMotionMagicVoltage.Velocity = ShooterConstants.pivotMaxVel;
                        }

                        shooterPivotLeftMotor.setControl(
                                        dynamicMotionMagicVoltage.withPosition(setpoint).withEnableFOC(true));
                }
        }

        @Override
        public void setShooterReference(double setpoint, boolean homed) {
                this.homed = homed;
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
                talonConfig.Feedback.SensorToMechanismRatio = 1.0;
                talonConfig.Feedback.RotorToSensorRatio = 90;
                talonConfig.Slot0.kP = p;
                talonConfig.Slot0.kI = i;
                talonConfig.Slot0.kD = d;

                dynamicMotionMagicVoltage.Acceleration = maxAccel;
                dynamicMotionMagicVoltage.Velocity = maxVel;

                shooterPivotLeftMotor.getConfigurator().apply(talonConfig);
        }

        @Override
        public void runPivotCharacterization(double volts) {
                shooterPivotLeftMotor.setVoltage(volts);
                shooterPivotRightMotor.setVoltage(volts);
        }
}

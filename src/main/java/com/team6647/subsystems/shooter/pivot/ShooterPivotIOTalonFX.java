package com.team6647.subsystems.shooter.pivot;

import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.andromedalib.sensors.SuperCANCoder;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.io.ObjectInputFilter.Status;

import com.andromedalib.motorControllers.SuperTalonFX;
import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;

public class ShooterPivotIOTalonFX implements ShooterPivotIO {

        private static SuperTalonFX shooterPivotLeftMotor = new SuperTalonFX(
                        ShooterConstants.shooterPivotLeftMotorID,
                        GlobalIdleMode.Brake);

        private static SuperTalonFX shooterPivotRightMotor = new SuperTalonFX(
                        ShooterConstants.shooterPivotRightMotorID,
                        GlobalIdleMode.Brake);

        private static SuperCANCoder shooterPivotEncoder;
        private static PositionVoltage positionVoltage = new PositionVoltage(0.0).withSlot(0);

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

        private static ArmFeedforward feedforward = new ArmFeedforward(ShooterConstants.pivotKs,
                        ShooterConstants.pivotKg, ShooterConstants.pivotKv);

        private double setpoint;

        public ShooterPivotIOTalonFX() {
                CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
                cancoderConfig.MagnetSensor.SensorDirection = ShooterConstants.shooterPivotEncoderInverted;
                cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
                cancoderConfig.MagnetSensor.MagnetOffset = ShooterConstants.shooterPivotEncoderOffset;
                shooterPivotEncoder = new SuperCANCoder(ShooterConstants.shooterPivotCANCoderID, cancoderConfig);

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
                                shooterPivotRightMotorTemperature);

                setPIDF(ShooterConstants.pivotKp, ShooterConstants.pivotKi, ShooterConstants.pivotKd, 0.0);

                shooterPivotLeftMotor.setControl(new Follower(ShooterConstants.shooterPivotRightMotorID, true));

        }

        @Override
        public void updateInputs(ShooterPivotIOInputs inputs) {
                BaseStatusSignal.refreshAll(
                                shooterPivotLeftMotorPosition,
                                shooterPivotRightMotorPosition,
                                shooterPivotLeftMotorVelocity,
                                shooterPivotRightMotorVelocity,
                                shooterPivotLeftMotorAppliedVolts,
                                shooterPivotRightMotorAppliedVolts,
                                cancoderAbsolutePosition,
                                cancoderAbsoluteVelocity);

                inputs.cancoderAbsolutePosition = cancoderAbsolutePosition.getValueAsDouble();

                inputs.cancoderAbsoluteVelocity = cancoderAbsoluteVelocity.getValueAsDouble();

                inputs.shooterPivotLeftMotorPosition = shooterPivotLeftMotorPosition.getValueAsDouble();

                inputs.shooterPivotRightMotorPosition = shooterPivotRightMotorPosition.getValueAsDouble();

                inputs.shooterPivotLeftMotorVelocity = shooterPivotLeftMotorVelocity.getValueAsDouble();

                inputs.shooterPivotRightMotorVelocity = shooterPivotRightMotorVelocity.getValueAsDouble();

                inputs.shooterPivotLeftMotorAppliedVolts = shooterPivotLeftMotorAppliedVolts.getValueAsDouble();

                inputs.shooterPivotRightMotorAppliedVolts = shooterPivotRightMotorAppliedVolts.getValueAsDouble();

                inputs.shooterPivotLeftMotorTemperatureCelsius = shooterPivotLeftMotorTemperature.getValueAsDouble();

                inputs.shooterPivotRightMotorTemperatureCelsius = shooterPivotRightMotorTemperature.getValueAsDouble();

                /* Sets pivot position based on setpoint */

                inputs.arbitraryFeedforward = feedforward
                                .calculate(Units.rotationsToRadians(setpoint - 0.3271484375),
                                                Units.rotationsToRadians(0.1) / 60);

                shooterPivotRightMotor.setControl(
                                positionVoltage.withPosition(setpoint).withFeedForward(inputs.arbitraryFeedforward));
        }

        @Override
        public void setShooterReference(double setpoint) {
                this.setpoint = setpoint / 360;
        }

        @Override
        public void disablePivot() {
                shooterPivotLeftMotor.disable();
                shooterPivotRightMotor.stopMotor();
        }

        @Override
        public void setPIDF(double p, double i, double d, double f) {
                TalonFXConfiguration talonConfig = new TalonFXConfiguration();
                talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                talonConfig.Slot0.kP = p;
                talonConfig.Slot0.kI = i;
                talonConfig.Slot0.kD = d;
                talonConfig.Slot0.kS = ShooterConstants.pivotKs;
                talonConfig.Slot0.kG = ShooterConstants.pivotKg;
                talonConfig.Slot0.kV = ShooterConstants.pivotKv;

                shooterPivotRightMotor.getConfigurator().apply(talonConfig);
        }

        @Override
        public void runPivotCharacterization(double volts) {
                shooterPivotLeftMotor.setVoltage(volts);
                shooterPivotRightMotor.setVoltage(volts);
        }
}

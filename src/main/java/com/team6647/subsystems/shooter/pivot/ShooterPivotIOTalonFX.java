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

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.andromedalib.motorControllers.SuperTalonFX;
import com.team6647.util.Constants.ShooterConstants;

public class ShooterPivotIOTalonFX implements ShooterPivotIO {

        private static SuperTalonFX shooterPivotLeftMotor = new SuperTalonFX(
                        ShooterConstants.shooterPivotLeftMotorID,
                        GlobalIdleMode.Brake,
                        false);

        private static SuperTalonFX shooterPivotRightMotor = new SuperTalonFX(
                        ShooterConstants.shooterPivotRightMotorID,
                        GlobalIdleMode.Brake,
                        true);

        private static SuperCANCoder shooterPivotEncoder;
        private static PositionVoltage positionVoltage = new PositionVoltage(0.0).withSlot(0);

        private static Follower follower = new Follower(ShooterConstants.shooterPivotLeftMotorID, true);

        private final StatusSignal<Double> shooterPivotLeftMotorPosition;
        private final StatusSignal<Double> shooterPivotRightMotorPosition;
        private final StatusSignal<Double> shooterPivotLeftMotorVelocity;
        private final StatusSignal<Double> shooterPivotRightMotorVelocity;
        private final StatusSignal<Double> shooterPivotLeftMotorAppliedVolts;
        private final StatusSignal<Double> shooterPivotRightMotorAppliedVolts;
        private final StatusSignal<Double> cancoderAbsolutePosition;
        private final StatusSignal<Double> cancoderAbsoluteVelocity;

        private double setpoint;

        public ShooterPivotIOTalonFX() {
                CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
                cancoderConfig.MagnetSensor.SensorDirection = ShooterConstants.shooterPivotEncoderInverted;
                cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
                shooterPivotEncoder = new SuperCANCoder(ShooterConstants.shooterPivotCANCoderID, cancoderConfig);

                shooterPivotLeftMotorPosition = shooterPivotLeftMotor.getPosition();
                shooterPivotRightMotorPosition = shooterPivotRightMotor.getPosition();
                shooterPivotLeftMotorVelocity = shooterPivotLeftMotor.getVelocity();
                shooterPivotRightMotorVelocity = shooterPivotRightMotor.getVelocity();
                shooterPivotLeftMotorAppliedVolts = shooterPivotLeftMotor.getMotorVoltage();
                shooterPivotRightMotorAppliedVolts = shooterPivotRightMotor.getMotorVoltage();
                cancoderAbsolutePosition = shooterPivotEncoder.getAbsolutePosition();
                cancoderAbsoluteVelocity = shooterPivotEncoder.getVelocity();

                BaseStatusSignal.setUpdateFrequencyForAll(50.0,
                                shooterPivotLeftMotorPosition,
                                shooterPivotRightMotorPosition,
                                shooterPivotLeftMotorVelocity,
                                shooterPivotRightMotorVelocity,
                                shooterPivotLeftMotorAppliedVolts,
                                shooterPivotRightMotorAppliedVolts,
                                cancoderAbsolutePosition,
                                cancoderAbsoluteVelocity);

                configMotor(ShooterConstants.pivotKp, ShooterConstants.pivotKi, ShooterConstants.pivotKd);
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

                inputs.cancoderAbsolutePosition.mut_replace(
                                cancoderAbsolutePosition.getValueAsDouble(),
                                Rotations);

                inputs.cancoderAbsoluteVelocity.mut_replace(
                                cancoderAbsoluteVelocity.getValueAsDouble(),
                                RotationsPerSecond);

                inputs.shooterPivotLeftMotorPosition.mut_replace(
                                shooterPivotLeftMotorPosition.getValueAsDouble(),
                                Rotations);

                inputs.shooterPivotRightMotorPosition.mut_replace(
                                shooterPivotRightMotorPosition.getValueAsDouble(),
                                Rotations);

                inputs.shooterPivotLeftMotorVelocity.mut_replace(
                                shooterPivotLeftMotorVelocity.getValueAsDouble(),
                                RotationsPerSecond);

                inputs.shooterPivotRightMotorVelocity.mut_replace(
                                shooterPivotRightMotorVelocity.getValueAsDouble(),
                                RotationsPerSecond);

                inputs.shooterPivotLeftMotorAppliedVolts.mut_replace(
                                shooterPivotLeftMotorAppliedVolts.getValueAsDouble(),
                                Volts);

                inputs.shooterPivotRightMotorAppliedVolts.mut_replace(
                                shooterPivotRightMotorAppliedVolts.getValueAsDouble(),
                                Volts);

                /* Sets pivot position based on setpoint */
                shooterPivotLeftMotor.setControl(positionVoltage.withPosition(0.4));
                shooterPivotRightMotor.setControl(follower);
        }

        @Override
        public void setShooterReference(double setpoint) {
                this.setpoint = setpoint;
        }

        @Override
        public void disablePivot() {
                shooterPivotLeftMotor.disable();
                shooterPivotRightMotor.stopMotor();
        }

        private void configMotor(double p, double i, double d) {
                TalonFXConfiguration talonConfig = new TalonFXConfiguration();
                talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                talonConfig.Slot0.kP = p;
                talonConfig.Slot0.kI = i;
                talonConfig.Slot0.kD = d;

                talonConfig.Feedback.FeedbackRemoteSensorID = ShooterConstants.shooterPivotCANCoderID;

                shooterPivotLeftMotor.getConfigurator().apply(talonConfig);
        }
}

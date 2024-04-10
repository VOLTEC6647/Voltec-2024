package com.team6647.subsystems.shooter.pivot;

import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.andromedalib.sensors.SuperCANCoder;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.andromedalib.motorControllers.SuperTalonFX;
import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.math.util.Units;

public class ShooterPivotIOTalonFX implements ShooterPivotIO {

        private static SuperTalonFX shooterPivotLeftMotor = new SuperTalonFX(
                        ShooterConstants.shooterPivotLeftMotorID,
                        GlobalIdleMode.Brake);

        private static SuperTalonFX shooterPivotRightMotor = new SuperTalonFX(
                        ShooterConstants.shooterPivotRightMotorID,
                        GlobalIdleMode.Brake);

        private static SuperCANCoder shooterPivotEncoder;
        private static MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

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
        private double horizontalPosition = -0.1552734375;
        private double maxGravityFF = ShooterConstants.pivotKf;

        public ShooterPivotIOTalonFX() {
                CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
                cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
                cancoderConfig.MagnetSensor.SensorDirection = ShooterConstants.shooterPivotEncoderInverted;
                cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
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

                shooterPivotLeftMotor.setControl(new Follower(ShooterConstants.shooterPivotRightMotorID, true));

                setPIDF(ShooterConstants.pivotKp, ShooterConstants.pivotKi, ShooterConstants.pivotKd, 0.0);
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
                                cancoderAbsoluteVelocity,
                                shooterPivotLeftMotorTemperature,
                                shooterPivotRightMotorTemperature,
                                shooterPivotLeftMotorCurrent,
                                shooterPivotRightMotorCurrent);

                inputs.cancoderAbsolutePosition = cancoderAbsolutePosition.getValueAsDouble() * 360;

                inputs.cancoderAbsoluteVelocity = cancoderAbsoluteVelocity.getValueAsDouble();

                inputs.shooterPivotLeftMotorPosition = shooterPivotLeftMotorPosition.getValueAsDouble();

                inputs.shooterPivotRightMotorPosition = shooterPivotRightMotorPosition.getValueAsDouble() * 360;

                inputs.shooterPivotLeftMotorVelocity = shooterPivotLeftMotorVelocity.getValueAsDouble();

                inputs.shooterPivotRightMotorVelocity = shooterPivotRightMotorVelocity.getValueAsDouble();

                inputs.shooterPivotLeftMotorAppliedVolts = shooterPivotLeftMotorAppliedVolts.getValueAsDouble();

                inputs.shooterPivotRightMotorAppliedVolts = shooterPivotRightMotorAppliedVolts.getValueAsDouble();

                inputs.shooterPivotLeftMotorTemperatureCelsius = shooterPivotLeftMotorTemperature.getValueAsDouble();

                inputs.shooterPivotRightMotorTemperatureCelsius = shooterPivotRightMotorTemperature.getValueAsDouble();

                inputs.shooterPivotLeftMotorCurrent = shooterPivotLeftMotorCurrent.getValueAsDouble();
                
                inputs.shooterPivotRightMotorCurrent = shooterPivotRightMotorCurrent.getValueAsDouble();

                inputs.inTolerance = Math.abs(cancoderAbsolutePosition.getValueAsDouble()
                                - setpoint) < ShooterConstants.positionTolerance;

                /* Sets pivot position based on setpoint */

                inputs.setpoint = setpoint * 360;

/*                 double rotations = (cancoderAbsolutePosition.getValueAsDouble() -
                                horizontalPosition);
                double radians = Units.rotationsToRadians(rotations);
                double cosineScalar = Math.cos(radians);

                inputs.arbitraryFeedforward = cosineScalar * maxGravityFF; */

                shooterPivotRightMotor.setControl(
                                motionMagicVoltage.withPosition(setpoint).withEnableFOC(true));
        }

        @Override
        public void setShooterReference(double setpoint) {
                this.setpoint = setpoint / 360;
        }

        @Override
        public void disablePivot() {
                shooterPivotLeftMotor.disable();
                shooterPivotRightMotor.disable();
                shooterPivotLeftMotor.stopMotor();
                shooterPivotRightMotor.stopMotor();
        }

        @Override
        public void setPIDF(double p, double i, double d, double f) {
                TalonFXConfiguration talonConfig = new TalonFXConfiguration();
                shooterPivotRightMotor.getConfigurator().apply(new TalonFXConfiguration());
                talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                talonConfig.Audio.BeepOnConfig = true;
                talonConfig.Feedback.FeedbackRemoteSensorID = shooterPivotEncoder.getDeviceID();
                talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                talonConfig.Feedback.SensorToMechanismRatio = 1.0;
                talonConfig.Feedback.RotorToSensorRatio = 90;
                talonConfig.Slot0.kP = p;
                talonConfig.Slot0.kI = i;
                talonConfig.Slot0.kD = d;

                talonConfig.MotionMagic.MotionMagicCruiseVelocity = 2; // Target cruise velocity of 80 rps
                talonConfig.MotionMagic.MotionMagicAcceleration = 10; // Target acceleration of 160 rps/s (0.5 seconds)

                maxGravityFF = f;

                shooterPivotRightMotor.getConfigurator().apply(talonConfig);
        }

        @Override
        public void runPivotCharacterization(double volts) {
                shooterPivotLeftMotor.setVoltage(volts);
                shooterPivotRightMotor.setVoltage(volts);
        }
}

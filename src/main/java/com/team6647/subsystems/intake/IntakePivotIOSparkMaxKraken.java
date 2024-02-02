/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 26 01 2024
 */

package com.team6647.subsystems.intake;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.andromedalib.motorControllers.SuperTalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakePivotIOSparkMaxKraken implements IntakePivotIO {

    private static SuperSparkMax leftIntakePivotMotor = new SuperSparkMax(
            IntakeConstants.intakePivotLeftMotorID,
            GlobalIdleMode.Coast,
            IntakeConstants.intakePivotLeftMotorInverted,
            IntakeConstants.intakeMotorsCurrentLimit,
            IntakeConstants.intakePivotEncoderPositionConversionFactor,
            IntakeConstants.intakePivotEncoderZeroOffset,
            IntakeConstants.intakePivotEncoderInverted);
    private static SuperTalonFX rightIntakePivotMotor = new SuperTalonFX(
            IntakeConstants.intakePivotRightMotorID,
            GlobalIdleMode.Coast,
            IntakeConstants.intakePivotRightMotorInverted);

    private static AbsoluteEncoder pivotEncoder;

    private static DigitalInput beamBrake = new DigitalInput(IntakeConstants.intakeBeamBrakeChannel);

    public IntakePivotIOSparkMaxKraken() {
        pivotEncoder = leftIntakePivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    }

    @Override
    public void updateInputs(IntakePivoIOInputs inputs) {
        inputs.intakePivotLeftMotorVelocity = leftIntakePivotMotor.getVelocity();
        inputs.intakePivotLeftMotorAppliedVoltage = leftIntakePivotMotor.getAppliedOutput();
        inputs.intakePivotLeftMotorPosition = leftIntakePivotMotor.getPosition();
        inputs.intakePivotLeftMotorCurrent = leftIntakePivotMotor.getOutputCurrent();

        inputs.intakePivotAbsoluteEncoderPosition = pivotEncoder.getPosition();

        inputs.intakePivotRightMotorVelocity = rightIntakePivotMotor.getVelocity().getValueAsDouble();
        inputs.intakePivotRightMotorAppliedVoltage = rightIntakePivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.intakePivotRightMotorPosition = rightIntakePivotMotor.getPosition().getValueAsDouble();
        inputs.intakePivtoRightMotorCurrent = rightIntakePivotMotor.getStatorCurrent().getValueAsDouble();

        inputs.intakeBeamBrake = beamBrake.get();
    }

    @Override
    public void setIntakeVoltage(double leftMotorVolts, double rightMotorVolts) {
        leftIntakePivotMotor.setVoltage(leftMotorVolts);
        rightIntakePivotMotor.setVoltage(rightMotorVolts);
    }
}
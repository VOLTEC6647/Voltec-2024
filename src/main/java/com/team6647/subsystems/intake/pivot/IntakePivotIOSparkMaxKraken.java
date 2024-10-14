/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 26 01 2024
 */

package com.team6647.subsystems.intake.pivot;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.andromedalib.motorControllers.SuperTalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.ctre.phoenix6.controls.VoltageOut;
import com.team6647.util.Constants.IntakeConstants;
import com.team6647.util.Constants.RobotConstants;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakePivotIOSparkMaxKraken implements IntakePivotIO {

    private static SuperSparkMax pushingPivotMotor = new SuperSparkMax(
            IntakeConstants.intakePushingMotor,
            GlobalIdleMode.Coast,
            IntakeConstants.intakePushingMotorInverted,
            IntakeConstants.intakeMotorsCurrentLimit,
            IntakeConstants.intakePivotEncoderPositionConversionFactor,
            IntakeConstants.intakePivotEncoderZeroOffset,
            IntakeConstants.intakePivotEncoderInverted);
    private static SuperTalonFX leftIntakePivotMotor = new SuperTalonFX(
            IntakeConstants.intakePivotLeftMotorID,
            GlobalIdleMode.Brake,
            IntakeConstants.intakePivotLeftMotorInverted,  RobotConstants.mechanismsCANnivore);
    private static SuperTalonFX rightIntakePivotMotor = new SuperTalonFX(
            IntakeConstants.intakePivotRightMotorID,
            GlobalIdleMode.Brake,
            IntakeConstants.intakePivotRightMotorInverted,  RobotConstants.mechanismsCANnivore);

    private static AbsoluteEncoder pivotEncoder;

    private SparkPIDController pushingController;

    private DigitalInput pushingLimitSwitch = new DigitalInput(IntakeConstants.pushingLimitSwitch);

    public IntakePivotIOSparkMaxKraken() {
        pushingController = pushingPivotMotor.getPIDController();

        pushingController.setP(0.8);

        pushingPivotMotor.setSmartCurrentLimit(10);
        pivotEncoder = pushingPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    }

    @Override
    public void updateInputs(IntakePivoIOInputs inputs) {
        inputs.intakePivotLeftMotorVelocity = leftIntakePivotMotor.getVelocity().getValueAsDouble();
        inputs.intakePivotLeftMotorAppliedVoltage = leftIntakePivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.intakePivotLeftMotorPosition = leftIntakePivotMotor.getPosition().getValueAsDouble();
        inputs.intakePivotLeftMotorCurrent = leftIntakePivotMotor.getStatorCurrent().getValueAsDouble();

        inputs.intakePushingMotorVelocity = pushingPivotMotor.getVelocity();
        inputs.intakePushingMotorAppliedVoltage = pushingPivotMotor.getAppliedOutput();
        inputs.intakePushingMotorPosition = pushingPivotMotor.getPosition();
        inputs.intakePushingMotorCurrent = pushingPivotMotor.getOutputCurrent();

        inputs.intakePivotAbsoluteEncoderPosition = pivotEncoder.getPosition();

        inputs.intakePivotRightMotorVelocity = rightIntakePivotMotor.getVelocity().getValueAsDouble();
        inputs.intakePivotRightMotorAppliedVoltage = rightIntakePivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.intakePivotRightMotorPosition = rightIntakePivotMotor.getPosition().getValueAsDouble();
        inputs.intakePivtoRightMotorCurrent = rightIntakePivotMotor.getStatorCurrent().getValueAsDouble();

        inputs.pushingLimitSwitchPressed = pushingLimitSwitch.get();
    }

    @Override
    public void setIntakeVoltage(double volts) {
        VoltageOut voltage = new VoltageOut(volts);
        leftIntakePivotMotor.setControl(voltage.withEnableFOC(true));
        rightIntakePivotMotor.setControl(voltage.withEnableFOC(true));
    }

    @Override
    public void setPushingReference(double position) {
        pushingController.setReference(position, ControlType.kPosition);
    }

    @Override
    public void setPushingPercentage(double percentagee) {
        pushingPivotMotor.set(percentagee);
    }

    @Override
    public void setPushingPosition(double position) {
        pushingPivotMotor.setPosition(position);
    }

    @Override
    public void disableIntake() {
        leftIntakePivotMotor.disable();
        rightIntakePivotMotor.disable();
    }
}

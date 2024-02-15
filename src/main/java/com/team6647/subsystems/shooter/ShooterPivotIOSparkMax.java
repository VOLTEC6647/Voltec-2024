/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.ShooterConstants;

public class ShooterPivotIOSparkMax implements ShooterPivotIO {

    private SuperSparkMax shooterPivotLeftMotor = new SuperSparkMax(
            ShooterConstants.shooterPivotMotorID,
            GlobalIdleMode.Brake, ShooterConstants.shooterPivotMotorInverted,
            ShooterConstants.shooterMotorCurrentLimit,
            ShooterConstants.armEncoderPositionConversionFactor,
            ShooterConstants.armEncoderZeroOffset,
            ShooterConstants.armEncoderInverted);

    private static AbsoluteEncoder pivotEncoder;
    private static SparkPIDController pivotController;

    public ShooterPivotIOSparkMax() {
        pivotEncoder = shooterPivotLeftMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pivotController = shooterPivotLeftMotor.getPIDController();

        pivotController.setFeedbackDevice(pivotEncoder);

        pivotController.setP(ShooterConstants.revPivotKp);
        pivotController.setP(ShooterConstants.revPivotKi);
        pivotController.setP(ShooterConstants.revPivotKd);
        pivotController.setP(ShooterConstants.revPivotKf);
    }

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        inputs.shooterAbsoluteEncoderPosition = pivotEncoder.getPosition();
        inputs.pivotMotorPosition = shooterPivotLeftMotor.getPosition();
        inputs.shooterAbsoluteEncoderVelocity = pivotEncoder.getVelocity();

        inputs.shooterPivotAppliedVolts = shooterPivotLeftMotor.getAppliedOutput()
                * shooterPivotLeftMotor.getBusVoltage();

    }

    @Override
    public void setShooterVoltage(double volts) {
        // shooterPivotLeftMotor.setVoltage(volts);
    }

    @Override
    public void setShooterReference(double setpoint) {
        pivotController.setReference(setpoint, ControlType.kPosition);
    }

    @Override
    public void setPID(double p, double i, double d, double f) {
        pivotController.setP(p);
        pivotController.setI(i);
        pivotController.setD(d);
        pivotController.setFF(f);
    }
}

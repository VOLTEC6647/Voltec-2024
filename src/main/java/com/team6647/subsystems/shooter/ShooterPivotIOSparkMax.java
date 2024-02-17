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
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.ShooterConstants;

public class ShooterPivotIOSparkMax implements ShooterPivotIO {

    private SuperSparkMax shooterPivotMotor = new SuperSparkMax(
            ShooterConstants.shooterPivotMotorID,
            GlobalIdleMode.Brake, ShooterConstants.shooterPivotMotorInverted,
            ShooterConstants.shooterMotorCurrentLimit,
            ShooterConstants.armEncoderPositionConversionFactor,
            ShooterConstants.armEncoderZeroOffset,
            ShooterConstants.armEncoderInverted);

    private static AbsoluteEncoder pivotEncoder;
    private static SparkPIDController pivotController;

    private double setpoint;

    public ShooterPivotIOSparkMax() {
        pivotEncoder = shooterPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pivotController = shooterPivotMotor.getPIDController();

        pivotController.setFeedbackDevice(pivotEncoder);

        pivotController.setP(ShooterConstants.pivotKp);
        pivotController.setP(ShooterConstants.pivotKi);
        pivotController.setP(ShooterConstants.pivotKd);
        pivotController.setP(ShooterConstants.pivotKf);

        pivotController.setReference(0, ControlType.kVelocity);

        shooterPivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ShooterConstants.pivotMaxPosition);
        shooterPivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ShooterConstants.pivotMinPosition);
    }

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        inputs.shooterAbsoluteEncoderPosition = pivotEncoder.getPosition();
        inputs.pivotMotorPosition = shooterPivotMotor.getPosition();
        inputs.shooterAbsoluteEncoderVelocity = pivotEncoder.getVelocity();

        inputs.shooterPivotAppliedVolts = shooterPivotMotor.getAppliedOutput()
                * shooterPivotMotor.getBusVoltage();

        inputs.inTolerance = Math
                .abs(shooterPivotMotor.getPosition() - setpoint) < ShooterConstants.positionTolerance;
    }

    @Override
    public void setShooterReference(double setpoint) {
        this.setpoint = setpoint;
        pivotController.setReference(setpoint, ControlType.kPosition);
    }

    @Override
    public void setPIDF(double p, double i, double d, double f) {
        pivotController.setP(p);
        pivotController.setI(i);
        pivotController.setD(d);
        pivotController.setFF(f);
    }

}

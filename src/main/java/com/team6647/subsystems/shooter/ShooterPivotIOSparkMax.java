/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.revrobotics.AbsoluteEncoder;
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

    public ShooterPivotIOSparkMax() {
        pivotEncoder = shooterPivotLeftMotor.getAbsoluteEncoder(Type.kDutyCycle);
    }

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        inputs.shooterAbsoluteEncoderPosition = pivotEncoder.getPosition();
        inputs.pivotMotorPosition = shooterPivotLeftMotor.getPosition();
    }

    @Override
    public void setShooterVoltage(double volts) {
        shooterPivotLeftMotor.setVoltage(volts);
    }
}

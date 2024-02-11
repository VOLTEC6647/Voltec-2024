/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 22 01 2024
 */
package com.team6647.subsystems.shooter;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.ShooterConstants;

public class ShooterIOSparkMax implements ShooterIO {

    private static SuperSparkMax rollerMotor = new SuperSparkMax(
            ShooterConstants.shooterRollerMotorID,
            GlobalIdleMode.Coast,
            true,
            ShooterConstants.rollerMotorCurrentLimit);

    private static SuperSparkMax topFlywheelMotor = new SuperSparkMax(
            ShooterConstants.flywheelTopMotorID,
            GlobalIdleMode.Coast,
            true,
            ShooterConstants.shooterMotorCurrentLimit);

    private static SuperSparkMax bottomFlywheelMotor = new SuperSparkMax(
            ShooterConstants.flywheelBottomMotorID,
            GlobalIdleMode.Coast,
            true,
            ShooterConstants.shooterMotorCurrentLimit);

    public ShooterIOSparkMax() {

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
    }

    @Override
    public void setShooterVelocity(double velocity) {

    }

    @Override
    public void setRollerVelocity(double velocity) {
        rollerMotor.set(velocity);
    }
}

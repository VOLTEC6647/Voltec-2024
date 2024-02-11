/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 22 01 2024
 */
package com.team6647.subsystems.shooter;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.IntakeConstants;
import com.team6647.util.Constants.ShooterConstants;

public class ShooterIOSparkMax implements ShooterIO {

    private static SuperSparkMax rollerMotor = new SuperSparkMax(
            ShooterConstants.shooterRollerMotorID,
            GlobalIdleMode.Coast,
            true,
            IntakeConstants.intakeMotorsCurrentLimit);

    public ShooterIOSparkMax() {

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        rollerMotor.set(0.25);
    }

    @Override
    public void setShooterVelocity(double velocity) {

    }
}

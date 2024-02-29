/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 16 09 2024
 */

package com.team6647.subsystems.shooter;

import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.andromedalib.motorControllers.SuperSparkMax;
import com.team6647.util.Constants.ShooterConstants;

public class ShooterIORollerSparkMax implements ShooterRollerIO {

    private static SuperSparkMax rollerMotor = new SuperSparkMax(
            ShooterConstants.shooterRollerMotorID,
            GlobalIdleMode.Brake,
            true,
            ShooterConstants.rollerMotorCurrentLimit);

    @Override
    public void updateInputs(ShooterRollerIOInputs inputs) {
        inputs.rollerVelocity = rollerMotor.getAppliedOutput();
        inputs.rollerCurrent = rollerMotor.getOutputCurrent();
    }

    @Override
    public void setRollerVelocity(double velocity) {
        rollerMotor.set(velocity);
    }
}

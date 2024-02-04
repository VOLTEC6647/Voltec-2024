/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.intake;

import com.andromedalib.motorControllers.SuperTalonFX;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {

    private SuperTalonFX intakeMotor = new SuperTalonFX(IntakeConstants.intakeMotorID, GlobalIdleMode.Coast, false);

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorAppliedVoltage = intakeMotor.getSupplyVoltage().getValueAsDouble();
        inputs.intakeMotorVelocity = intakeMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setIntakeVelocity(double velocity) {
        intakeMotor.set(velocity);
    }
}

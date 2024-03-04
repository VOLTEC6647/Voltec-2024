/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 01 2024s
 */

package com.team6647.subsystems.intake;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO {
    private SuperSparkMax intakeMotor = new SuperSparkMax(IntakeConstants.intakeMotorID, false);

    private DigitalInput intakeBeamBrake = new DigitalInput(IntakeConstants.intakeBeamBrakeChannel);

    public IntakeIOSparkMax() {
        intakeMotor.setSmartCurrentLimit(5);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorAppliedVoltage = intakeMotor.getBusVoltage();
        inputs.intakeMotorVelocity = intakeMotor.getVelocity();
        inputs.intakeMotorCurrent = intakeMotor.getOutputCurrent();
        inputs.intakeBeamBrake = intakeBeamBrake.get();
    }

    @Override
    public void setIntakeVelocity(double velocity) {
        intakeMotor.set(velocity);
    }
}

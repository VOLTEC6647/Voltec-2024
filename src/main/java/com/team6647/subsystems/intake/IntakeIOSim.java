/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 29 01 2024
 */

package com.team6647.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private DCMotorSim simMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorAppliedVoltage = 0.0;
        inputs.intakeMotorVelocity = simMotor.getAngularVelocityRPM();
    }

    @Override
    public void setIntakeVelocity(double velocity) {
        simMotor.setInput(velocity);
    }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 01 2024
 */

package com.team6647.subsystems.elevator;

import com.andromedalib.math.Conversions;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim elevatorMotorSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);

    double appliedVolts = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorMotorSim.update(LOOP_PERIOD_SECS);

        inputs.elevatorPosition = Conversions.rotationsToMeters(elevatorMotorSim.getAngularPositionRotations(), 1);
        inputs.elevatorAbsoluteEncoderPosition = 0.0; // TODO SET

        inputs.topBeambrake = false; // TODO SET
        inputs.bottomBeamBrake = false; // TODO SET
        inputs.elevatorAppliedVolts = appliedVolts;

    }

    @Override
    public void setElevatorVoltage(double volts) {
        appliedVolts = volts;
        elevatorMotorSim.setInputVoltage(volts);
    }

}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 01 2024
 */

package com.team6647.subsystems.elevator;

import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.andromedalib.motorControllers.SuperSparkMax;
import com.team6647.util.Constants.ElevatorConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
    private static SuperSparkMax elevatorMotor = new SuperSparkMax(
            ElevatorConstants.elevatorMotorID,
            GlobalIdleMode.brake,
            ElevatorConstants.elevatorMotorInverted,
            ElevatorConstants.elevatorMotorCurrentLimit,
            ElevatorConstants.elevatorAbsoluteEncoderPositionConversionFactor,
            ElevatorConstants.elevatorAbsoluteEncoderZeroOffset,
            ElevatorConstants.elevatorAbsoluteEncoderInverted);

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorPosition = elevatorMotor.getPosition();
        inputs.elevatorAbsoluteEncoderPosition = 0.0; // TODO SET

        inputs.topBeambrake = false; // TODO SET
        inputs.bottomBeamBrake = false; // TODO SET
        inputs.elevatorAppliedVolts = elevatorMotor.getBusVoltage();
    }

    @Override
    public void setElevatorVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.intake.roller;

import com.andromedalib.motorControllers.SuperTalonFX;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOTalonFX implements IntakeIO {

    private SuperTalonFX intakeMotor = new SuperTalonFX(IntakeConstants.intakeMotorID, GlobalIdleMode.Coast, false);

    private DigitalInput intakeBeamBrake = new DigitalInput(IntakeConstants.intakeBeamBrakeChannel);

    public IntakeIOTalonFX() {
        int angleContinuousCurrentLimit = 5;
        int anglePeakCurrentLimit = 10;
        double anglePeakCurrentDuration = 0.1;
        boolean angleEnableCurrentLimit = true;

        double openLoopRamp = 0.5;
        double closedLoopRamp = 0.0;

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = angleContinuousCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = angleEnableCurrentLimit;
        config.CurrentLimits.SupplyCurrentThreshold = anglePeakCurrentLimit;
        config.CurrentLimits.SupplyTimeThreshold = anglePeakCurrentDuration;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorAppliedVoltage = intakeMotor.getSupplyVoltage().getValueAsDouble();
        inputs.intakeMotorVelocity = intakeMotor.getVelocity().getValueAsDouble();
        inputs.intakeMotorCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();

        inputs.intakeBeamBrake = intakeBeamBrake.get();
    }

    @Override
    public void setIntakeVelocity(double velocity) {
        intakeMotor.set(velocity);
    }
}

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

/*     private static Rev2mDistanceSensor distMXP;
 */
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

        /* distMXP = new Rev2mDistanceSensor(Port.kMXP);
        distMXP.setAutomaticMode(true); */
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorAppliedVoltage = intakeMotor.getSupplyVoltage().getValueAsDouble();
        inputs.intakeMotorVelocity = intakeMotor.getVelocity().getValueAsDouble();
        inputs.intakeMotorCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();

        inputs.intakeBeamBrake = intakeBeamBrake.get();
        inputs.intakeMXPRange = 0.0;
        // ONLY UNCOMMENT WHEN USING TOF SENSOR
        /*
         * if (distMXP.isRangeValid()) {
         * inputs.intakeMXPRange = distMXP.getRange();
         * inputs.intakeBeamBrake = !(distMXP.getRange() < 5);
         * } else {
         * inputs.intakeBeamBrake = false;
         * inputs.intakeMXPRange = 0.0;
         * 
         * }
         */
    }

    @Override
    public void setIntakeVelocity(double velocity) {
        intakeMotor.set(velocity);
    }
}

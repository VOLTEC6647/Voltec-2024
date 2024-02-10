/**
 * Written by Juan Pablo Gutiérrez 
 * 
 * 26 01 2024
 */

package com.team6647.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakePivotIOSim implements IntakePivotIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim leftMotorSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);
    private DCMotorSim rightMotorSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);

    @Override
    public void updateInputs(IntakePivoIOInputs inputs) {
        leftMotorSim.update(LOOP_PERIOD_SECS);
        rightMotorSim.update(LOOP_PERIOD_SECS);

        inputs.intakePivotLeftMotorAppliedVoltage = 0.0;
        inputs.intakePivotLeftMotorPosition = leftMotorSim.getAngularPositionRotations();
        inputs.intakePivotLeftMotorVelocity = leftMotorSim.getAngularVelocityRPM();

        inputs.intakePivotAbsoluteEncoderPosition = leftMotorSim.getAngularPositionRotations();

        inputs.intakePivotRightMotorAppliedVoltage = 0.0;
        inputs.intakePivotRightMotorPosition = rightMotorSim.getAngularPositionRotations();
        inputs.intakePivotRightMotorVelocity = rightMotorSim.getAngularVelocityRPM();
    }

    @Override
    public void setIntakeVoltage(double leftMotorVolts, double rightMotorVolts) {
        leftMotorSim.setInputVoltage(leftMotorVolts);
        rightMotorSim.setInputVoltage(rightMotorVolts);
    }
}

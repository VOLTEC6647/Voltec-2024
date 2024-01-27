/**
 * Written by Juan Pablo Gutiérrez 
 * 
 * 26 01 2024
 */

package com.team6647.subsystems.intake;

import com.andromedalib.math.Conversions;
import com.team6647.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import com.team6647.util.Constants.ElevatorConstants;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    public void setIntakeVoltage(double voltage) {
        leftMotorSim.setInputVoltage(voltage);
        rightMotorSim.setInputVoltage(voltage);
    }
}

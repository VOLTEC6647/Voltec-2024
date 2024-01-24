/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 01 2024
 */

package com.team6647.subsystems.elevator;

import com.andromedalib.math.Conversions;
import com.team6647.util.Constants.ElevatorConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim elevatorMotorSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);

    private ProfiledPIDController elevatorPIDController = new ProfiledPIDController(ElevatorConstants.elevatorKp,
            ElevatorConstants.elevatorKi, ElevatorConstants.elevatorKd, new TrapezoidProfile.Constraints(1, 1));

    private double elevatorAppliedVolts = 0.0;
    private double setpoint = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorMotorSim.update(LOOP_PERIOD_SECS);

        inputs.elevatorPosition = Conversions.rotationsToMeters(elevatorMotorSim.getAngularPositionRotations(), 1);
        inputs.elevatorAbsoluteEncoderPosition = 0.0; // TODO SET

        inputs.topBeambrake = false; // TODO SET
        inputs.bottomBeamBrake = false; // TODO SET
        inputs.elevatorAppliedVolts = elevatorAppliedVolts;

        moveElevator();
    }

    @Override
    public void setElevatorPosition(double position) {
        setpoint = position;
    }

    private void moveElevator() {
        double volts = elevatorPIDController
                .calculate(Conversions.rotationsToMeters(elevatorMotorSim.getAngularPositionRotations(), 1), setpoint);
        elevatorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        elevatorMotorSim.setInputVoltage(volts);
    }
}

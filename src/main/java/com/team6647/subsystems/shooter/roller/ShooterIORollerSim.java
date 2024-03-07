/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 16 02 2024
 */

package com.team6647.subsystems.shooter.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIORollerSim implements ShooterRollerIO {
    private DCMotorSim rollerMotorSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);

    @Override
    public void updateInputs(ShooterRollerIOInputs inputs) {
        inputs.rollerVelocity = rollerMotorSim.getAngularVelocityRPM();
    }

    @Override
    public void setRollerVelocity(double velocity) {
        rollerMotorSim.setInputVoltage(velocity * 12);
    }

}

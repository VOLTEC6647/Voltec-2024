/*+
 * Written by Juan Pablo Gutiérrez
 * 
 * 10 02 2024
 */
package com.team6647.subsystems.shooter.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterPivotIOSim implements ShooterPivotIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim pivotMotorSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        pivotMotorSim.update(LOOP_PERIOD_SECS);

        inputs.shooterAbsoluteEncoderPosition = pivotMotorSim.getAngularPositionRotations();
        inputs.pivotMotorPosition = pivotMotorSim.getAngularPositionRotations();
    }

}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 10 02 2024
 */

package com.team6647.subsystems.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim topMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);
    private DCMotorSim bottomMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.025);

    private double setpoint = 0.0;

    private SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.01);
    private PIDController flywheelFeedback = new PIDController(0.6, 0.0, 0.0);

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        topMotor.update(LOOP_PERIOD_SECS);
        bottomMotor.update(LOOP_PERIOD_SECS);
    
        inputs.topMotorVelocity = topMotor.getAngularVelocityRPM();
        inputs.bottomMotorVelocity = bottomMotor.getAngularVelocityRPM();

        topMotor.setInputVoltage(flywheelFeedback.calculate(topMotor.getAngularVelocityRPM(), setpoint)
                + flywheelFeedforward.calculate(setpoint));
        bottomMotor.setInputVoltage(flywheelFeedback.calculate(bottomMotor.getAngularVelocityRPM(), setpoint)
                + flywheelFeedforward.calculate(setpoint));
    }

    @Override
    public void setShooterVelocity(double velocity) {
        setpoint = velocity;
    }
}

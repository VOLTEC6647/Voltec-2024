// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.andromedalib.andromedaSwerve.andromedaModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AndromedaModuleIOSim implements AndromedaModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim driveSim = new DCMotorSim(DCMotor.getFalcon500(1), 6.75, 0.025);
    private DCMotorSim turnSim = new DCMotorSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.004);

    private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    @Override
    public void updateInputs(AndromedaModuleIOInputs inputs) {
        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);

        inputs.drivePosition = driveSim.getAngularPositionRad();
        inputs.driveVelocity = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;

        inputs.encoderAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
        inputs.steerAngle = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnAppliedVolts = turnAppliedVolts;
    }

    @Override
    public void setDriveVoltage(double volts, boolean internalControl) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setTurnVoltage(double volts, boolean internalControl) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }
}

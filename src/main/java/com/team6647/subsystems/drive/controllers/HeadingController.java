/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 03 2024
 */

package com.team6647.subsystems.drive.controllers;

import org.littletonrobotics.junction.AutoLogOutput;

import com.team6647.util.LoggedTunableNumber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import lombok.Setter;

public class HeadingController {
    private LoggedTunableNumber kp = new LoggedTunableNumber("HeadingController/kp", 3.5);
    private LoggedTunableNumber ki = new LoggedTunableNumber("HeadingController/ki", 0.0);
    private LoggedTunableNumber kd = new LoggedTunableNumber("HeadingController/kd", 0.00001);
    private LoggedTunableNumber maxSpeed = new LoggedTunableNumber("HeadingController/maxSpeed", 50);
    private LoggedTunableNumber maxAcceleration = new LoggedTunableNumber("HeadingController/maxAcceleration", 50);
    private LoggedTunableNumber headingTolerance = new LoggedTunableNumber("HeadingController/tolerance", 1);

    private ProfiledPIDController headingController;
    @Setter
    @AutoLogOutput
    public Rotation2d targetHeading = new Rotation2d();

    public HeadingController() {
        headingController = new ProfiledPIDController(kp.get(), ki.get(), kd.get(),
                new TrapezoidProfile.Constraints(maxSpeed.get(), maxAcceleration.get()));
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(headingTolerance.get());
    }

    public double update(Rotation2d currentHeading) {
        LoggedTunableNumber.ifChanged(hashCode(), pid -> {
            headingController.setP(pid[0]);
            headingController.setI(pid[1]);
            headingController.setD(pid[2]);
            headingController.setConstraints(new TrapezoidProfile.Constraints(pid[3], pid[4]));
        }, kp, ki, kd, maxSpeed, maxAcceleration);

        return headingController.calculate(currentHeading.getRadians(), targetHeading.getRadians());
    }

    public boolean inTolerance() {
        return headingController.atGoal();
    }
}

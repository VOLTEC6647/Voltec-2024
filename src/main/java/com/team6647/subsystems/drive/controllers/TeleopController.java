/*+
 * Written by Juan Pablo Gutiérrez
 * 
 * 23 03 2024
 */

package com.team6647.subsystems.drive.controllers;

import com.andromedalib.math.Functions;
import com.team6647.util.LoggedTunableNumber;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.OperatorConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TeleopController {
    
    private LoggedTunableNumber deadband = new LoggedTunableNumber("TeleopController/Deadband", OperatorConstants.controllerDeadband);

    private double controllerX = 0.0;
    private double controllerY = 0.0;
    private double controllerOmega = 0.0;
    private boolean fieldRelative = true;

    public void acceptControllerInput(double x, double y, double omega, boolean fieldRelative) {
        this.controllerX = x;
        this.controllerY = y;
        this.controllerOmega = omega;
        this.fieldRelative = fieldRelative;
    }

    public ChassisSpeeds update(Rotation2d currentAngle) {
        controllerX = Functions.handleDeadband(controllerX, deadband.get());
        controllerY = Functions.handleDeadband(controllerY, deadband.get());
        controllerOmega = Functions.handleDeadband(controllerOmega, deadband.get());

        Translation2d translation = new Translation2d(controllerX, controllerY)
                .times(DriveConstants.andromedaSwerveConfig.maxSpeed);
        double rotation = controllerOmega * DriveConstants.andromedaSwerveConfig.maxAngularVelocity;

        return fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                        currentAngle)
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }
}

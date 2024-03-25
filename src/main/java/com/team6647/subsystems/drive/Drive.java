/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 20 03 2024
 */

package com.team6647.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModuleIO;
import com.andromedalib.andromedaSwerve.andromedaModule.GyroIO;
import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig;
import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.team6647.subsystems.drive.controllers.HeadingController;
import com.team6647.subsystems.drive.controllers.TeleopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Setter;

public class Drive extends AndromedaSwerve {

    private static Drive instance;

    @Setter
    @AutoLogOutput(key = "Swerve/mMode")
    private DriveMode mDriveMode = DriveMode.TELEOP;

    private ChassisSpeeds desiredChassisSpeeds;

    private TeleopController teleopController = new TeleopController();
    private HeadingController headingController = new HeadingController();

    private Drive(GyroIO gyroIO, AndromedaModuleIO[] modulesIO, AndromedaSwerveConfig andromedaProfile) {
        super(gyroIO, modulesIO, andromedaProfile);
    }

    public static Drive getInstance(GyroIO gyroIO, AndromedaModuleIO[] modulesIO,
            AndromedaSwerveConfig andromedaProfile) {
        if (instance == null) {
            instance = new Drive(gyroIO, modulesIO, andromedaProfile);
        }
        return instance;
    }

    @Override
    public void periodic() {
        super.periodic();

        ChassisSpeeds teleopSpeeds = teleopController.update(getSwerveAngle());
        switch (mDriveMode) {
            case TELEOP:
                desiredChassisSpeeds = teleopSpeeds;
                break;
            case HEADING_LOCK:
                teleopSpeeds.omegaRadiansPerSecond = headingController.update(getSwerveAngle());
                desiredChassisSpeeds = teleopSpeeds;
                break;
            case PATH_FOLLOWING:
                break;
        }

        drive(desiredChassisSpeeds);
    }

    public enum DriveMode {
        TELEOP,
        HEADING_LOCK,
        PATH_FOLLOWING,
    }

    /**
     * Drives the robot with the given translation and rotation
     * 
     * @param translation   Translation movement
     * @param rotation      Desired rotation
     * @param fieldRelative True if field relative
     */
    public void acceptTeleopInputs(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier rotation, BooleanSupplier fieldOrientedControl) {
        teleopController.acceptControllerInput(translationY.getAsDouble(), translationX.getAsDouble(),
                rotation.getAsDouble(), fieldOrientedControl.getAsBoolean());
    }

    /**
     * Drives the robot with the given translation and rotation
     * 
     * @param translation   Translation movement
     * @param rotation      Desired rotation
     * @param fieldRelative True if field relative
     */
    public void acceptTeleopInputs(Translation2d translation, DoubleSupplier rotation,
            BooleanSupplier fieldOrientedControl) {
        teleopController.acceptControllerInput(translation.getY(), translation.getX(),
                rotation.getAsDouble(), fieldOrientedControl.getAsBoolean());
    }

    public void setTargetHeading(Rotation2d heading) {
        headingController.setTargetHeading(heading);
    }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 15 02 2024
 */

package com.team6647.util;

import com.team6647.util.Constants.FieldConstants;
import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootingCalculatorUtil {

    public record ShootingParameters(
            double robotToSpeakerDistance,
            ChassisSpeeds robotSpeed,
            Rotation2d robotAngle,
            double pivotAngle,
            double flywheelRPM) {
    }

    public ShootingParameters calculateShootingParameters(Pose2d robotPositionField, ChassisSpeeds currentRobotSpeed,
            Rotation2d robotAngle) {

        Rotation2d speakerToRobotAngle = robotPositionField.getTranslation()
                .minus(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()))
                .getAngle();

        Pose2d futurePose = new Pose2d(
                robotPositionField.getTranslation().plus(
                        new Translation2d(currentRobotSpeed.vxMetersPerSecond * 0.5,
                                currentRobotSpeed.vyMetersPerSecond * 0.5)),
                robotPositionField.getRotation().plus(robotAngle));

        // Calculate the robot to speaker distance
        double robotToSpeakerDistance = futurePose.getTranslation()
                .getDistance(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()));

        double flywheelRPM = ShooterConstants.shooterRPMMap.get(robotToSpeakerDistance);
        double pivotAngle = ShooterConstants.shooterPivotMap.get(robotToSpeakerDistance);

        return new ShootingParameters(robotToSpeakerDistance, currentRobotSpeed, speakerToRobotAngle, pivotAngle,
                flywheelRPM);
    }
}

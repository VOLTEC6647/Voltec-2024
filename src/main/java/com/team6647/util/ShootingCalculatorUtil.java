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
                        Rotation2d robotAngle,
                        double pivotAngle,
                        double flywheelRPM) {
        }

        /*
         * public static ShootingParameters calculateShootingParameters(Pose2d
         * robotPositionField,
         * ChassisSpeeds currentRobotSpeed,
         * Rotation2d robotAngle) {
         * 
         * Rotation2d speakerToRobotAngle = robotPositionField.getTranslation()
         * .minus(AllianceFlipUtil
         * .apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()))
         * .getAngle();
         * 
         * Pose2d futurePose = new Pose2d(
         * robotPositionField.getTranslation().plus(
         * new Translation2d(currentRobotSpeed.vxMetersPerSecond * 0.5,
         * currentRobotSpeed.vyMetersPerSecond * 0.5)),
         * robotPositionField.getRotation().plus(robotAngle));
         * 
         * // Calculate the robot to speaker distance
         * double robotToSpeakerDistance = futurePose.getTranslation()
         * .getDistance(AllianceFlipUtil
         * .apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()));
         * 
         * double flywheelRPM =
         * ShooterConstants.shooterRPMMap.get(robotToSpeakerDistance);
         * double pivotAngle =
         * ShooterConstants.shooterPivotMap.get(robotToSpeakerDistance);
         * 
         * return new ShootingParameters(currentRobotSpeed, speakerToRobotAngle,
         * pivotAngle,
         * flywheelRPM);
         * }
         */

        /*
         * public static ShootingParameters getShootingParameters(Pose2d robotPose2d,
         * ChassisSpeeds currentRobotSpeed) {
         * Translation2d robotTranslation = calculateShootingWhileDriving(robotPose2d,
         * currentRobotSpeed);
         * 
         * Rotation2d speakerToRobotAngle = robotPose2d.getTranslation()
         * .minus(AllianceFlipUtil
         * .apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()))
         * .getAngle();
         * 
         * Translation2d speakerPosition =
         * AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening
         * .toTranslation2d());
         * 
         * return new ShootingParameters(robotTranslation, speakerToRobotAngle,
         * ShooterConstants.shooterPivotMap.get(speakerPosition.getDistance(
         * robotTranslation)),
         * ShooterConstants.shooterRPMMap.get(speakerPosition.getDistance(
         * robotTranslation)));
         * }
         */

        public static ShootingParameters getShootingParameters(Pose2d robotPose, Translation2d targetlocation) {

                Translation2d robotToTarget = targetlocation.minus(robotPose.getTranslation());

                double distance = targetlocation.getDistance(robotPose.getTranslation());

                Rotation2d angle = robotToTarget.getAngle();

                return new ShootingParameters(angle, ShooterConstants.shooterPivotMap.get(distance),
                                ShooterConstants.shootingRPM);

        }

        public static Translation2d calculateShootingWhileDriving(Pose2d robotPositionField,
                        ChassisSpeeds currentRobotSpeed) {

                Translation2d speakerPosition = AllianceFlipUtil
                                .apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());

                double distanceToTarget = speakerPosition.getDistance(robotPositionField.getTranslation());

                double shotTime = ShooterConstants.shooterTimeMap.get(distanceToTarget);

                Translation2d movingGoalLocation, testGoalLocation;

                movingGoalLocation = new Translation2d();

                for (int i = 0; i < 5; i++) {
                        double virtualGoalX = speakerPosition.getX()
                                        - shotTime * (currentRobotSpeed.vxMetersPerSecond * 0.5);
                        double virtualGoalY = speakerPosition.getY()
                                        - shotTime * (currentRobotSpeed.vyMetersPerSecond * 0.5);
                        testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

                        double distanceToTestGoal = testGoalLocation.getDistance(robotPositionField.getTranslation());
                        double newShotTime = ShooterConstants.shooterTimeMap.get(distanceToTestGoal);

                        if (Math.abs(newShotTime - shotTime) <= 0.010) {
                                i = 4;
                        }

                        if (i == 4) {
                                movingGoalLocation = testGoalLocation;
                        } else {
                                shotTime = newShotTime;
                        }
                }

                return movingGoalLocation;
        }
}

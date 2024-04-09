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

        public static ShootingParameters getShootingParameters(Pose2d robotPose, Translation2d targetlocation) {

                Translation2d robotToTarget = targetlocation.minus(robotPose.getTranslation());

                double distance = targetlocation.getDistance(robotPose.getTranslation());

                Rotation2d angle = robotToTarget.getAngle().rotateBy(Rotation2d.fromDegrees(180));

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

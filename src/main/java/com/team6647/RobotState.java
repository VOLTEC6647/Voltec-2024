/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 09 04 2024
 * 
 * Manages all odometry and pose estimation for the robot
 */

package com.team6647;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.odometry.SuperRobotState;
import com.team6647.util.AllianceFlipUtil;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.FieldConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotState extends SuperRobotState {

    private static RobotState instance;

    private static SwerveDrivePoseEstimator poseEstimator;

    private final Vector<N3> stateStandardDeviations = VecBuilder.fill(0.04, 0.04,
            edu.wpi.first.math.util.Units.degreesToRadians(1));

    private static SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4];
    private static Rotation2d lastGyroAngle = new Rotation2d();

    private RobotState() {
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.andromedaSwerveConfig.swerveKinematics,
                new Rotation2d(),
                new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                }, new Pose2d(), stateStandardDeviations, VecBuilder.fill(0.2, 0.2,
                        edu.wpi.first.math.util.Units.degreesToRadians(100)));
    }

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    @Override
    public void periodic() {
        double robotToSpeakerDistance = getPose().getTranslation()
                .getDistance(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()));

        Logger.recordOutput("Odometry/RobotSpeakerDistane", robotToSpeakerDistance);
    }

    @Override
    public void addOdometryObservations(double currentTimeSencods, Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions) {
        lastModulePositions = modulePositions;

        if (AllianceFlipUtil.shouldFlip()) {
            gyroAngle = gyroAngle.rotateBy(Rotation2d.fromDegrees(-180));
        }

        lastGyroAngle = gyroAngle;

        poseEstimator.updateWithTime(currentTimeSencods, gyroAngle, modulePositions);
    }

    /**
     * Gets the current robot's position on the field
     * 
     * @return {@link Pose2d} of the robot
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public static Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the robot's position on the field
     * 
     * @param pose2d New position
     */
    public static void resetPose(Pose2d pose2d) {
        poseEstimator.resetPosition(lastGyroAngle, lastModulePositions, pose2d);
    }

    /**
     * Adds a vision measurement to the pose estimator
     * 
     * @param observedPose     The pose of the vision target
     * @param timestampLatency The latency of the vision system
     */
    public static void addVisionMeasurements(Pose2d observedPose, double timestampLatency) {
        poseEstimator.addVisionMeasurement(observedPose, timestampLatency);
    }

    /**
     * Adds a vision measurement to the pose estimator
     * 
     * @param observedPose       The pose of the vision target
     * @param timestampLatency   The latency of the vision system
     * @param standardDeviations The standard deviations of the vision measurements
     */
    public static void addVisionMeasurements(Pose2d observedPose, double timestampLatency,
            Vector<N3> standardDeviations) {
/* 
        if (AllianceFlipUtil.shouldFlip()) {
            observedPose = new Pose2d(observedPose.getX(), observedPose.getY(),
                    observedPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        } */

        poseEstimator.addVisionMeasurement(observedPose, timestampLatency, standardDeviations);
    }
}

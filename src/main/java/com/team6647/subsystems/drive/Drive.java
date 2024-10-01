/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 20 03 2024
 */

package com.team6647.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModuleIO;
import com.andromedalib.andromedaSwerve.andromedaModule.GyroIO;
import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig;
import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.team6647.RobotState;
import com.team6647.subsystems.drive.controllers.HeadingController;
import com.team6647.subsystems.drive.controllers.TeleopController;
import com.team6647.util.Constants.DriveConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import lombok.Setter;

public class Drive extends AndromedaSwerve {

    private static Drive instance;

    @Setter
    @AutoLogOutput(key = "Swerve/mMode")
    public static DriveMode mDriveMode = DriveMode.TELEOP;

    private ChassisSpeeds desiredChassisSpeeds;

    private TeleopController teleopController = new TeleopController();
    private HeadingController headingController = new HeadingController();

    private Drive(GyroIO gyroIO, AndromedaModuleIO[] modulesIO, AndromedaSwerveConfig andromedaProfile) {
        super(gyroIO, modulesIO, andromedaProfile, RobotState.getInstance());
        AutoBuilder.configureHolonomic(
                RobotState::getPose,
                RobotState::resetPose,
                this::getRobotRelativeChassisSpeeds,
                this::acceptInputs,
                DriveConstants.holonomicPathConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, this);

        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });
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
            DoubleSupplier rotation, BooleanSupplier fieldOrientedControl, BooleanSupplier moveStraight) {                
        teleopController.acceptControllerInput(translationY.getAsDouble(), translationX.getAsDouble(),
                rotation.getAsDouble(), fieldOrientedControl.getAsBoolean(), moveStraight.getAsBoolean());
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
                rotation.getAsDouble(), fieldOrientedControl.getAsBoolean(), false);
    }

    private void acceptInputs(ChassisSpeeds speeds) {
        setMDriveMode(DriveMode.PATH_FOLLOWING);
        desiredChassisSpeeds = speeds;
    }

    public void setTargetHeading(Rotation2d heading) {
        headingController.setTargetHeading(heading);
    }

    /**
     * Retuns a command to drive to a target pose
     * 
     * @param targetPose  Target pose
     * @param constraints Path constraints
     * @return Command to drive to target pose
     */
    public Command getPathFindPath(Pose2d targetPose, PathConstraints constraints) {
        return Commands.sequence(
                new InstantCommand(() -> {
                    mDriveMode = DriveMode.PATH_FOLLOWING;
                }),
                AutoBuilder.pathfindToPose(
                        targetPose,
                        constraints,
                        0.0,
                        0.0));
    }

    public Command getPathFindThenFollowPath(String pathName, PathConstraints constraints) {
        return Commands.sequence(
                new InstantCommand(() -> {
                    mDriveMode = DriveMode.PATH_FOLLOWING;
                }),
                AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(pathName), constraints));
    }

    /**
     * Returns true if the robot is within the heading tolerance
     * 
     * @return True if the robot is within the heading tolerance
     */
    public boolean headingInTolerance() {
        return headingController.inTolerance();
    }
}

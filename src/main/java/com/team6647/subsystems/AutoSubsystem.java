/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.subsystems;

import org.littletonrobotics.junction.Logger;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.pathplanner.lib.auto.AutoBuilder;
import com.team6647.util.Constants.DriveConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSubsystem extends SubsystemBase {
  private static AutoSubsystem instance;

  private final AndromedaSwerve andromedaSwerve;

  private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  private AprilTagFieldLayout aprilTagFieldLayout;
  private Transform3d cameraTransformRobotRelative;

  /** Creates a new AutoSubsystem. */
  private AutoSubsystem(AndromedaSwerve swerve) {
    this.andromedaSwerve = swerve;

    AutoBuilder.configureHolonomic(
        swerve::getPose,
        swerve::resetPose,
        swerve::getRelativeChassisSpeeds,
        swerve::drive,
        DriveConstants.holonomicPathConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, andromedaSwerve);

    try {
      aprilTagFieldLayout = new AprilTagFieldLayout("getName()");
    } catch (Exception e) {
    }
  }

  public static AutoSubsystem getInstance(AndromedaSwerve swerve) {
    if (instance == null) {
      instance = new AutoSubsystem(swerve);
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Transform3d getTargetPose() {
    return new Transform3d();
  }

  SwerveDrivePoseEstimator poseEstimator;

  public void x() {
    // Obtain ID from limelight API
    Integer id = Integer.parseInt(limelightTable.getEntry("tid").getString(""));

    // Get Tag Pose from file
    Pose3d tagPose = aprilTagFieldLayout.getTagPose(id) == null ? aprilTagFieldLayout.getTagPose(id).get()
        : new Pose3d();

    // Gets the Tag's position (camera relative)
    Transform3d targetPose = getTargetPose();
    // Compute the camera position in relation to the tag/field
    Pose3d cameraPose = tagPose.transformBy(targetPose.inverse());

    // Transform the camera's position by the transform (offset) from the robot.
    // Gives the robot's position
    Pose3d robotPose = cameraPose.transformBy(cameraTransformRobotRelative);

    // Adds the estimated robot's position to the EKF to fuse with odometry
    poseEstimator.addVisionMeasurement(robotPose.toPose2d(), Logger.getRealTimestamp());
  }
}

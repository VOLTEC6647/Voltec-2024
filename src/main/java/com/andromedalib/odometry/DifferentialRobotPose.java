/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.odometry;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO: Not ready for use
/**
 * Class used to calculate the robot's position on the field
 * through the use of an unfiltered Kalman filter in the
 * {@link DifferentialDrivePoseEstimator} class
 * 
 */
public class DifferentialRobotPose extends SubsystemBase {
  static DifferentialDrivePoseEstimator poseEstimator;

  static Field2d field2d;

  /**
   * Creates a new DifferentialRobotPose.
   */
  public DifferentialRobotPose() {
  }

  @Override
  public void periodic() {
    field2d.setRobotPose(getEstimatedPosition());
    SmartDashboard.putData(field2d);
  }

  /**
   * Publishes data and information to SmartDashboard. Override this method to
   * include custom information
   */
  public void outputTelemetry() {}

  /**
   * Gets the estimated position of the robot
   * 
   * @return the estimated position of the robot
   */
  public Pose2d getEstimatedPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the pose estimator
   * 
   * @return the pose estimator
   */
  public DifferentialDrivePoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  public Field2d getField2d() {
    return field2d;
  }

  public void updatePoseEstimator(Rotation2d gyroAngle, DifferentialDriveWheelSpeeds wheelSpeeds, double leftDistance,
      double rightDistance) {

    // poseEstimator.update(gyroAngle, wheelSpeeds, leftDistance, rightDistance);

  }

}

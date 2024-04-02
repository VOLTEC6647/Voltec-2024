/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 25 01 2024
 */
package com.team6647.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import com.team6647.subsystems.drive.Drive;
import com.team6647.util.Constants.FieldConstants;
import com.team6647.util.Constants.VisionConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private static VisionSubsystem instance;

  private VisionIO io;
  private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double zMargin = 0.75;

  /** Creates a new VisionSubsystem. */
  private VisionSubsystem(VisionIO io) {
    this.io = io;

    changePipeline(VisionConstants.odometryPipelineNumber);
  }

  public static VisionSubsystem getInstance(VisionIO io) {
    if (instance == null) {
      instance = new VisionSubsystem(io);
    }

    return instance;
  }

  @Override
  public synchronized void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    computeVisionMeasurements();
  }

  public void computeVisionMeasurements() {
    if (inputs.hasTarget) {
      Pose3d robotPose3d = new Pose3d(inputs.estimatedRobotPose);

      // Exit if robot pose is off the field
      if (robotPose3d.getX() < -fieldBorderMargin
          || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
          || robotPose3d.getY() < -fieldBorderMargin
          || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
          || robotPose3d.getZ() < -zMargin
          || robotPose3d.getZ() > zMargin) {
        return;
      }

      // Exit if ambiguity is too high
      if (!(inputs.bestTargetPoseAmbiguity < ambiguityThreshold)) {
        return;
      }

      if (checkTagDistance(inputs.targetsIDs.length, inputs.bestTransform3d, 1, 3.5)
          || checkTagDistance(inputs.targetsIDs.length, inputs.bestTransform3d, 2, 6.0)
          || checkTagDistance(inputs.targetsIDs.length, inputs.bestTransform3d, 3, 8.0)) {
        Drive.addVisionMeasurements(inputs.estimatedRobotPose, ambiguityThreshold);
      }
    }
  }

  public boolean hasTarget() {
    return inputs.hasTarget;
  }

  public boolean hasTargetID(int ID) {
    return inputs.hasTarget && inputs.bestTargetTagID == ID;
  }

  public void changePipeline(int pipelineNumber) {
    io.changePipeline(pipelineNumber);
  }

  public double getTY() {
    return 0.0;
    // return inputs.TY;
  }

  public double getTX() {
    return 0.0;
    // return inputs.TX;
  }

  // Check if distance between robot and tag is less than a certain value ;)
  public boolean checkTagDistance(int targetSize, Transform3d bestTransform, int numberOfTags, double distance) {
    if (targetSize == numberOfTags) {
      if (bestTransform.getTranslation()
          .getDistance(new Translation3d(0, 0, 0)) < distance) {
        return true;
      }
    }

    return false;
  }

}

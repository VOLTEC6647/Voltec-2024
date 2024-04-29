/**
 * Written by Juan Pablo Gutiérrez
 */

package com.team6647.subsystems.vision;

import org.photonvision.PhotonUtils;

import com.andromedalib.vision.LimelightHelpers;
import com.team6647.util.AllianceFlipUtil;
import com.team6647.util.Constants.VisionConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionIOLimelight implements VisionIO {

    AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public VisionIOLimelight() {
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs, Rotation2d robotHeading, double headingVelocity) {
        inputs.currentPipelineNumber = (int) LimelightHelpers.getCurrentPipelineIndex(VisionConstants.aprilLimeNTName);

        if (AllianceFlipUtil.shouldFlip()) {
            robotHeading = robotHeading.rotateBy(Rotation2d.fromDegrees(-180));
        }

        LimelightHelpers.Results result = LimelightHelpers
                .getLatestResults(VisionConstants.aprilLimeNTName).targetingResults;

        inputs.TA = LimelightHelpers.getTA(VisionConstants.aprilLimeNTName);

        if (!(result.botpose[0] == 0 && result.botpose[1] == 0) &&
                LimelightHelpers.getTA(VisionConstants.aprilLimeNTName) > 0.1) {
            inputs.hasTarget = true;

            boolean doRejectUpdate = false;
            LimelightHelpers.SetRobotOrientation(VisionConstants.aprilLimeNTName,
                    robotHeading.getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.aprilLimeNTName);
            if (Math.abs(headingVelocity) > 720) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                inputs.estimatedPose2d = mt2.pose;
                inputs.timestampLatency = mt2.timestampSeconds;
            }

            inputs.TX = LimelightHelpers.getTX(VisionConstants.aprilLimeNTName);
            inputs.TY = LimelightHelpers.getTY(VisionConstants.aprilLimeNTName);
            inputs.targetDistance = computeTagDistance(inputs.estimatedPose2d);

            try {
                inputs.targetID = (int) LimelightHelpers.getFiducialID(VisionConstants.aprilLimeNTName);
            } catch (Exception e) {
                DriverStation.reportError("[Limelight] Error found while trying to compute target ID", true);
                inputs.targetID = 0;
            }
        } else {
            inputs.estimatedPose2d = new Pose2d();
            inputs.timestampLatency = 0;
            inputs.hasTarget = false;
            inputs.targetDistance = 0.0;
            inputs.targetID = 0;
        }
    }

    public double computeTagDistance(Pose2d pose) {
        double id = LimelightHelpers.getFiducialID(VisionConstants.aprilLimeNTName);

        if (layout.getTagPose((int) id).isPresent()) {
            return PhotonUtils.getDistanceToPose(LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.aprilLimeNTName),
                    layout.getTagPose((int) id).get().toPose2d());

        }

        return 0.0;

    }

    public static boolean isRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return true;
    }

    @Override
    public void changePipeline(int pipelineNumber) {
        LimelightHelpers.setPipelineIndex(VisionConstants.aprilLimeNTName, pipelineNumber);
    }

    @Override
    public void setForceBlink() {
        LimelightHelpers.setLEDMode_ForceBlink(VisionConstants.aprilLimeNTName);
    }

    @Override
    public void setForceOff() {
        LimelightHelpers.setLEDMode_ForceOff(VisionConstants.aprilLimeNTName);
    }
}
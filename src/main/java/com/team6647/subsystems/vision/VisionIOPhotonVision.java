/**
 * Written by Juan Pablo Gutiérrez
 */

package com.team6647.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.andromedalib.math.GeomUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonVision implements VisionIO {
    private PhotonCamera camera = new PhotonCamera("Limelight3_back");
    private AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    private Transform3d cameraToRobot = new Transform3d(); /* Todo set */
    private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraToRobot);

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            inputs.hasTarget = true;

            PhotonTrackedTarget target = result.getBestTarget();

            inputs.bestTargetYaw = target.getYaw();
            inputs.bestTargetPitch = target.getPitch();
            inputs.bestTargetArea = target.getArea();
            inputs.bestTransform3d = target.getBestCameraToTarget();
            inputs.bestTargetTagID = target.getFiducialId();
            inputs.bestTargetPoseAmbiguity = target.getPoseAmbiguity();
            inputs.alternateTranform3d = target.getAlternateCameraToTarget();
            inputs.observedPose2d = GeomUtil.toPose2d(GeomUtil.toTransform2d(inputs.bestTransform3d));

            inputs.bestTargetObservedRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                    target.getBestCameraToTarget(),
                    layout.getTagPose(target.getFiducialId()).get(), cameraToRobot);

            EstimatedRobotPose robotPose = getEstimatedGlobalPose(result).get();

            inputs.estimatedRobotPose = robotPose.estimatedPose.toPose2d();
            inputs.estimatedTimestamp = robotPose.timestampSeconds;

            inputs.targetsIDs = getTargetIDs(result);

            inputs.targetDistance = PhotonUtils.getDistanceToPose(inputs.observedPose2d,
                    layout.getTagPose(target.getFiducialId()).get().toPose2d());
        } else {
            inputs.hasTarget = true;
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPipelineResult result) {
        return poseEstimator.update(result);
    }

    public int[] getTargetIDs(PhotonPipelineResult result) {
        return result.getTargets().stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
    }
}

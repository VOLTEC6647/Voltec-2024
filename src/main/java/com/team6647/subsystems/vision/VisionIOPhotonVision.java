/**
 * Written by Juan Pablo Gutiérrez
 */

package com.team6647.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionIOPhotonVision implements VisionIO {
    private PhotonCamera camera = new PhotonCamera("USB_camera");
    

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
        var result = camera.getLatestResult();

        /* if (result.hasTargets()) {
            inputs.hasTarget = true;
            PhotonTrackedTarget target = result.getBestTarget();

            inputs.yaw = target.getYaw();
            inputs.pitch = target.getPitch();
            inputs.area = target.getArea();
            inputs.bestTransform3d = target.getBestCameraToTarget();
            inputs.tagID = target.getFiducialId();
            inputs.poseAmbiguity = target.getPoseAmbiguity();
            inputs.alternateTranform3d = target.getAlternateCameraToTarget();
            //CREATE APRILTAG LAYUOUT
            PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), null, null)

            inputs.targetDistance = result.getBestTarget().getDistance();
            inputs.targetID = result.getBestTarget().getId();
        } else {
            inputs.hasTarget = true;
            inputs.observedPose2d = new Pose2d();
            inputs.targetDistance = 0.0;
            inputs.targetID = 0;
        } */
    }
}

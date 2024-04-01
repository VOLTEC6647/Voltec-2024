/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 25 01 2024
 */

package com.team6647.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean hasTarget = false;
        public double bestTargetYaw = 0.0;
        public double bestTargetPitch = 0.0;
        public double bestTargetArea = 0.0;
        public Transform3d bestTransform3d = new Transform3d();
        public Transform3d alternateTranform3d = new Transform3d();
        public int bestTargetTagID = 0;
        public double bestTargetPoseAmbiguity = 0.0;
        public Pose3d bestTargetObservedRobotPose = new Pose3d();
        public Pose2d estimatedRobotPose = new Pose2d();
        public double estimatedTimestamp = 0.0;

        public int[] targetsIDs;

        public int currentPipelineNumber = 0;
        public Pose2d observedPose2d = new Pose2d();
        public double timestampLatency = 0.0;
        public double targetDistance = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {
    }

    public default void changePipeline(int pipelineNumber) {

    }
}

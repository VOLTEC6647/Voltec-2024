/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 25 01 2024
 */

package com.team6647.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean hasTarget = false;
        public double yaw = 0.0;
        public double pitch = 0.0;
        public double area = 0.0;
        public Transform3d bestTransform3d = new Transform3d();
        public Transform3d alternateTranform3d = new Transform3d();
        public int tagID = 0;
        public double poseAmbiguity = 0.0;

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

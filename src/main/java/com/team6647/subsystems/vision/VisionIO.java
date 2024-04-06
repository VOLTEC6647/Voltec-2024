/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 25 01 2024
 */

package com.team6647.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public double TA = 0.0;
        public double TY = 0.0;
        public double TX = 0.0;
        public int currentPipelineNumber = 0;
        public Pose2d observedPose2d = new Pose2d();
        public double timestampLatency = 0.0;
        public boolean hasTarget = false;
        public double targetDistance = 0.0;
        public int targetID = 0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {
    }

    public default void changePipeline(int pipelineNumber) {

    }

    public default void setForceBlink() {

    }

    public default void setForceOff(){
        
    }

}
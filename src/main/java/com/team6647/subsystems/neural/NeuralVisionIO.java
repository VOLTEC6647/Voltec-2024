/**
 * Written by JUan Pablo Gutiérrez
 * 
 * 20 02 2024
 */

package com.team6647.subsystems.neural;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface NeuralVisionIO {

    @AutoLog
    public static class NeuralVisionIOInputs {
        public double TA = 0.0;
        public double TY = 0.0;
        public double TX = 0.0;
        public String tclass = "";
        public double timestampLatency = 0.0;
        public boolean hasTarget = false;
    }

    public default void updateInputs(NeuralVisionIOInputs inputs) {
    }
    public default void changePipeline(int pipelineNumber) {
    }

}

/**
 * Written by JUan Pablo Gutiérrez
 * 
 * 20 02 2024
 */

package com.team6647.subsystems.neural;

import org.littletonrobotics.junction.AutoLog;

public interface NeuralVisionIO {

    @AutoLog
    public static class NeuralVisionIOInputs {
        public double TA = 0.0;
        public double TY = 0.0;
        public double TX = 0.0;
        public double timestampLatency = 0.0;
        public boolean hasTarget = false;
        public double confidence;
        public int numTargets;
    }

    public default void updateInputs(NeuralVisionIOInputs inputs) {
    }
}

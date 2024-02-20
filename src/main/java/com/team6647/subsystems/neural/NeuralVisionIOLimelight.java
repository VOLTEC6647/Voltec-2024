/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 20 02 2024
 */

package com.team6647.subsystems.neural;

import com.andromedalib.vision.LimelightHelpers;
import com.team6647.util.Constants.VisionConstants;

public class NeuralVisionIOLimelight implements NeuralVisionIO {

    @Override
    public void updateInputs(NeuralVisionIOInputs inputs) {

        inputs.TA = LimelightHelpers.getTA(VisionConstants.neuralLimeNTName);
        inputs.TY = LimelightHelpers.getTA(VisionConstants.neuralLimeNTName);
        inputs.TX = LimelightHelpers.getTA(VisionConstants.neuralLimeNTName);
        inputs.hasTarget = (inputs.TA > 1) ? true : false;
    }
}

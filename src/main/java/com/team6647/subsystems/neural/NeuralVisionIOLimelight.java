/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 20 02 2024
 */

package com.team6647.subsystems.neural;

import com.andromedalib.vision.LimelightHelpers;
import com.team6647.util.Constants.VisionConstants;

import edu.wpi.first.wpilibj.DriverStation;

public class NeuralVisionIOLimelight implements NeuralVisionIO {

    @Override
    public void updateInputs(NeuralVisionIOInputs inputs) {
        if(DriverStation.isAutonomous()||true){
            LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(VisionConstants.neuralLimeNTName);
            inputs.TA = LimelightHelpers.getTA(VisionConstants.neuralLimeNTName);
            inputs.TY = LimelightHelpers.getTY(VisionConstants.neuralLimeNTName);
            inputs.TX = LimelightHelpers.getTX(VisionConstants.neuralLimeNTName);
            inputs.hasTarget = (inputs.TA != 0) ? true : false;
            if(inputs.hasTarget){
                //inputs.confidence = results.targetingResults.targets_Detector[0].confidence;
                

            }else{

            }
            
            
        }
    }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 20 02 2024
 */

package com.team6647.subsystems.neural;

import org.littletonrobotics.junction.Logger;

import com.andromedalib.vision.LimelightHelpers;
import com.team6647.util.Constants.VisionConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeuralVisionSubsystem extends SubsystemBase {

  private static NeuralVisionSubsystem instance;

  private NeuralVisionIO io;
  private NeuralVisionIOInputsAutoLogged inputs = new NeuralVisionIOInputsAutoLogged();

  /** Creates a new NeuralVisionSubsystem. */
  private NeuralVisionSubsystem(NeuralVisionIO io) {
    this.io = io;

    changePipeline(VisionConstants.odometryPipelineNumber);
  }

  public static NeuralVisionSubsystem getInstance(NeuralVisionIO io) {
    if (instance == null) {
      instance = new NeuralVisionSubsystem(io);
    }

    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    Logger.recordOutput("IsAutonomous", DriverStation.isAutonomous());
  }

  public boolean hasTarget() {
    return inputs.hasTarget;
  }

  public double getTY() {
    return inputs.TY;
  }

  public double getTX() {
    return inputs.TX;
  }
  
  private void changePipeline(int pipelineNumber) {
    io.changePipeline(pipelineNumber);
  }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 25 01 2024
 */
package com.team6647.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionAutoSubsystem extends SubsystemBase {

  private static VisionAutoSubsystem instance;

  private VisionIO io;
  private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  /** Creates a new VisionSubsystem. */
  private VisionAutoSubsystem(VisionIO io, AndromedaSwerve swerve) {
    this.io = io;
  }

  public static VisionAutoSubsystem getInstance(VisionIO io, AndromedaSwerve swerve) {
    if (instance == null) {
      instance = new VisionAutoSubsystem(io, swerve);
    }

    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    computeVisionMeasurements();
  }

  public void computeVisionMeasurements() {
    if (inputs.hasTarget) {
      if (inputs.targetDistance < 1.5) {
        AndromedaSwerve.addVisionMeasurements(inputs.observedPose2d, inputs.timestampLatency);
      }
    }
  }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 25 01 2024
 */
package com.team6647.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.team6647.util.Constants.VisionConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private static VisionSubsystem instance;

  private VisionIO io;
  private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  /** Creates a new VisionSubsystem. */
  private VisionSubsystem(VisionIO io) {
    this.io = io;

    changePipeline(VisionConstants.odometryPipelineNumber);
  }

  public static VisionSubsystem getInstance(VisionIO io) {
    if (instance == null) {
      instance = new VisionSubsystem(io);
    }

    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    computeVisionMeasurements();
    Logger.recordOutput("IsAutonomous", DriverStation.isAutonomous());
  }

  public void computeVisionMeasurements() {
    if (inputs.hasTarget) {
      if (inputs.targetDistance < 5) {

        double xyStdDev = 0.125
            * Math.pow(inputs.targetDistance, 2.0)
            / inputs.idCount == 0 ? 1 : inputs.idCount;

        Vector<N3> visionmeasurementStandardDeviations = VecBuilder.fill(xyStdDev, xyStdDev,
            edu.wpi.first.math.util.Units.degreesToRadians(50));

        Logger.recordOutput("Vision/StandarDev", xyStdDev);

        AndromedaSwerve.addVisionMeasurements(inputs.observedPose2d, inputs.timestampLatency,
            visionmeasurementStandardDeviations);
      }
    }
  }

  public void setLimelightMode(int mode) {
    if (mode == 2) {
      io.setForceBlink();
    } else if (mode == 1) {
      io.setForceOff();
    }
  }

  public boolean hasTarget() {
    return inputs.hasTarget;
  }

  public boolean hasTargetID(int ID) {
    return inputs.hasTarget && inputs.targetID == ID;
  }

  public void changePipeline(int pipelineNumber) {
    io.changePipeline(pipelineNumber);
  }

  public double getTY() {
    return inputs.TY;
  }

  public double getTX() {
    return inputs.TX;
  }
}
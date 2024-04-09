/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 25 01 2024
 */
package com.team6647.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.team6647.RobotContainer;
import com.team6647.RobotState;
import com.team6647.util.Constants.VisionConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private static VisionSubsystem instance;
  private static AndromedaSwerve andromedaSwerve = RobotContainer.andromedaSwerve;

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
    io.updateInputs(inputs, andromedaSwerve.getSwerveAngle(), andromedaSwerve.getHeadingVelocity());
    Logger.processInputs("Vision", inputs);
    computeVisionMeasurements();
    Logger.recordOutput("IsAutonomous", DriverStation.isAutonomous());
  }

  public void computeVisionMeasurements() {
    if (inputs.hasTarget) {
      if (inputs.targetDistance < 5) {

        RobotState.addVisionMeasurements(inputs.estimatedPose2d, inputs.timestampLatency,
            VecBuilder.fill(.6, .6, 9999999));
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
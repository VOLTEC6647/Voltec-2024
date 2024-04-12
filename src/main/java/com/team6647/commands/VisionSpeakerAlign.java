/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 10 03 2024
 */
package com.team6647.commands;

import org.littletonrobotics.junction.Logger;

import com.team6647.RobotState;
import com.team6647.subsystems.SuperStructure;
import com.team6647.subsystems.drive.Drive;
import com.team6647.subsystems.drive.Drive.DriveMode;
import com.team6647.subsystems.vision.VisionSubsystem;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.FieldConstants.Speaker;
import com.team6647.util.Constants.VisionConstants;
import com.team6647.util.AllianceFlipUtil;
import com.team6647.util.ShootingCalculatorUtil;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionSpeakerAlign extends Command {
  private Drive swerve;
  private VisionSubsystem visionSubsystem;

  private int stageID;
  private Translation2d speakerPose;

  private ShootingParameters parameters;

  private double targetigVel = 0.0;

  public VisionSpeakerAlign(Drive swevre, VisionSubsystem visionSubsystem) {
    this.swerve = swevre;
    this.visionSubsystem = visionSubsystem;

    addRequirements(visionSubsystem);
  }

  @Override
  public void initialize() {
    visionSubsystem.changePipeline(VisionConstants.speakerPipelineNumber);
    speakerPose = AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.toTranslation2d());
    stageID = AllianceFlipUtil.shouldFlip() ? VisionConstants.speakerRedCenterTagID
        : VisionConstants.speakerBlueCenterTagID;
    Logger.recordOutput("VisionSpeakerAlign/SpeakerPose", speakerPose);

  }

  @Override
  public void execute() {
    this.parameters = ShootingCalculatorUtil.getShootingParameters(RobotState.getPose(),
        speakerPose);

    SuperStructure.updateShootingParameters(parameters);

    if (visionSubsystem.hasTargetID(stageID)) {
      double kP = .005;
      targetigVel = visionSubsystem.getTX() * kP;

      targetigVel *= DriveConstants.maxAngularVelocityRadsPerSec;

      targetigVel *= -1.0;

      swerve.acceptTeleopInputs(() -> 0, () -> 0, () -> targetigVel, () -> false);
      swerve.setMDriveMode(DriveMode.TELEOP);
    } else {
      swerve.setMDriveMode(DriveMode.HEADING_LOCK);

      swerve.setTargetHeading(parameters.robotAngle());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    visionSubsystem.changePipeline(VisionConstants.odometryPipelineNumber);

    swerve.setMDriveMode(DriveMode.TELEOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerve.headingInTolerance();
  }
}

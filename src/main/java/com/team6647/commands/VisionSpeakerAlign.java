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

  private Translation2d speakerPose;

  private ShootingParameters parameters;


  public VisionSpeakerAlign(Drive swevre, VisionSubsystem visionSubsystem) {
    this.swerve = swevre;
    this.visionSubsystem = visionSubsystem;

    addRequirements(visionSubsystem, swerve);
  }

  @Override
  public void initialize() {
    visionSubsystem.changePipeline(VisionConstants.speakerPipelineNumber);
    speakerPose = AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.toTranslation2d());
    Logger.recordOutput("VisionSpeakerAlign/SpeakerPose", speakerPose);

  }

  @Override
  public void execute() {
    this.parameters = ShootingCalculatorUtil.getShootingParameters(RobotState.getPose(),
        speakerPose);

    SuperStructure.updateShootingParameters(parameters);

    Drive.setMDriveMode(DriveMode.HEADING_LOCK);

    swerve.setTargetHeading(parameters.robotAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    visionSubsystem.changePipeline(VisionConstants.odometryPipelineNumber);

    Drive.setMDriveMode(DriveMode.TELEOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerve.headingInTolerance();
  }
}

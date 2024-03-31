/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 30 03 2024
 */
package com.team6647.commands;

import org.littletonrobotics.junction.Logger;

import com.team6647.subsystems.drive.Drive;
import com.team6647.subsystems.drive.Drive.DriveMode;
import com.team6647.subsystems.vision.VisionSubsystem;
import com.team6647.util.AllianceFlipUtil;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.FieldConstants;
import com.team6647.util.Constants.VisionConstants;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionAmpAlign extends Command {

  private Drive swerve;
  private VisionSubsystem visionSubsystem;

  private int ampID;
  private Translation2d ampPose;

  private double targetigVel = 0.0;

  /** Creates a new VisionAmpAlign. */
  public VisionAmpAlign(Drive swerve, VisionSubsystem visionSubsystem) {
    this.swerve = swerve;
    this.visionSubsystem = visionSubsystem;

    ampPose = AllianceFlipUtil.apply(FieldConstants.amp).getTranslation();
    ampID = AllianceFlipUtil.shouldFlip() ? VisionConstants.ampRedTagID
        : VisionConstants.ampBlueTagID;
    Logger.recordOutput("VisionAmpAlign/AmpPose", ampPose);

    addRequirements(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionSubsystem.changePipeline(VisionConstants.speakerPipelineNumber);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (visionSubsystem.hasTargetID(ampID)) {
      double kP = .005;
      targetigVel = visionSubsystem.getTX() * kP;

      targetigVel *= DriveConstants.maxAngularVelocity;

      targetigVel *= -1.0;

      swerve.acceptTeleopInputs(() -> 0, () -> 0, () -> targetigVel, () -> false);
      swerve.setMDriveMode(DriveMode.TELEOP);
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
    return visionSubsystem.getTY() < -12;
  }
}

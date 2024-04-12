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

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionAmpAlign extends Command {

  private Drive swerve;
  private VisionSubsystem visionSubsystem;

  private int ampID;
  private Translation2d ampPose;

  private double targetigVel = 0.0;
  private double rangeVelocity = 0.0;

  /** Creates a new VisionAmpAlign. */
  public VisionAmpAlign(Drive swerve, VisionSubsystem visionSubsystem) {
    this.swerve = swerve;
    this.visionSubsystem = visionSubsystem;

    addRequirements(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ampPose = AllianceFlipUtil.apply(FieldConstants.amp).getTranslation();
    ampID = AllianceFlipUtil.shouldFlip() ? VisionConstants.ampRedTagID
        : VisionConstants.ampBlueTagID;
    Logger.recordOutput("VisionAmpAlign/AmpPose", ampPose);

    Logger.recordOutput("VisionAmpAlign/AmpID", ampID);

    visionSubsystem.changePipeline(VisionConstants.ampPipelineNumber);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (visionSubsystem.hasTargetID(ampID)) {
      double kP = .00075;
      targetigVel = visionSubsystem.getTX() * kP;

      targetigVel *= DriveConstants.maxAngularVelocityRadsPerSec;

      targetigVel *= -1.0;

      swerve.setMDriveMode(DriveMode.TELEOP);

      double rangeKp = .0025;
      rangeVelocity = visionSubsystem.getTY() * rangeKp;
      rangeVelocity *= DriveConstants.maxSpeedMetersPerSecond;
      rangeVelocity *= -1.0;

      Logger.recordOutput("VisionAmpAlign/Aiming", targetigVel);
      Logger.recordOutput("VisionAmpAlign/Range", rangeVelocity);

      swerve.acceptTeleopInputs(() -> rangeVelocity, () -> 0, () -> targetigVel, () -> false);
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
    return false;
  }
}

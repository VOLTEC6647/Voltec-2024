// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647.commands;

import org.littletonrobotics.junction.Logger;

import com.team6647.RobotState;
import com.team6647.subsystems.SuperStructure;
import com.team6647.subsystems.drive.Drive;
import com.team6647.subsystems.drive.Drive.DriveMode;
import com.team6647.subsystems.vision.VisionSubsystem;
import com.team6647.util.AllianceFlipUtil;
import com.team6647.util.Constants.FieldConstants;
import com.team6647.util.Constants.FieldConstants.Speaker;
import com.team6647.util.ShootingCalculatorUtil;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionShuttleAlign extends Command {

  private Drive swerve;
  private VisionSubsystem visionSubsystem;

  private Translation2d speakerPose;

  private ShootingParameters parameters;

  /** Creates a new VisionShuttleAlign. */
  public VisionShuttleAlign(Drive swevre, VisionSubsystem visionSubsystem) {
    this.swerve = swevre;
    this.visionSubsystem = visionSubsystem;

    addRequirements(visionSubsystem, swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionSubsystem.setLimelightMode(2);
    speakerPose = AllianceFlipUtil.apply(FieldConstants.shuttlePose.getTranslation());
    Logger.recordOutput("VisionShuttle/SpeakerPose", speakerPose);

  }

  // Called every time the scheduler runs while the command is scheduled.
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
    visionSubsystem.setLimelightMode(1);

    Drive.setMDriveMode(DriveMode.TELEOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerve.headingInTolerance();
  }
}

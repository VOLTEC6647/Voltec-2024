/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 10 03 2024
 */
package com.team6647.commands;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.andromedalib.util.AllianceFlipUtil;
import com.team6647.subsystems.SuperStructure;
import com.team6647.subsystems.vision.VisionSubsystem;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.FieldConstants.Speaker;
import com.team6647.util.Constants.VisionConstants;
import com.team6647.util.ShootingCalculatorUtil;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionSpeakerAlign extends Command {

  private AndromedaSwerve swerve;
  private VisionSubsystem visionSubsystem;

  private int stageID;

  private ShootingParameters parameters;

  private double targetigVel = 0.2;

  /** Creates a new VisionSpeakerAlign. */
  public VisionSpeakerAlign(AndromedaSwerve swevre, VisionSubsystem visionSubsystem) {
    this.swerve = swevre;
    this.visionSubsystem = visionSubsystem;

    addRequirements(swerve, visionSubsystem);

    stageID = AllianceFlipUtil.shouldFlip() ? VisionConstants.speakerRedCentgerTagID
        : VisionConstants.speakerBlueCenterTagID;

    SmartDashboard.putNumber("STAGE", stageID);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionSubsystem.changePipeline(VisionConstants.speakerPipelineNumber);

    this.parameters = ShootingCalculatorUtil.getShootingParameters(swerve.getPose(),
        AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.toTranslation2d()));

    SuperStructure.updateShootingParameters(parameters);

    swerve.setHeadingOverride(true);
    swerve.setTargetHeading(parameters.robotAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (visionSubsystem.hasTargetID(stageID)) {
      // kP (constant of proportionality)
      // this is a hand-tuned number that determines the aggressiveness of our
      // proportional control loop
      // if it is too high, the robot will oscillate around.
      // if it is too low, the robot will never reach its target
      // if the robot never turns in the correct direction, kP should be inverted.
      double kP = .005;

      // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
      // rightmost edge of
      // your limelight 3 feed, tx should return roughly 31 degrees.
      double targetingAngularVelocity = visionSubsystem.getTX() * kP;

      // convert to radians per second for our drive method
      targetingAngularVelocity *= DriveConstants.maxAngularVelocity;

      // invert since tx is positive when the target is to the right of the Fcrosshair
      targetingAngularVelocity *= -1.0;

      targetigVel = targetingAngularVelocity;

      swerve.drive(new Translation2d(), targetingAngularVelocity, false);
    } else {
      swerve.drive(new ChassisSpeeds());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    visionSubsystem.changePipeline(VisionConstants.odometryPipelineNumber);
    swerve.setHeadingOverride(false);
    swerve.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  swerve.angleInTolerance();
  }
}

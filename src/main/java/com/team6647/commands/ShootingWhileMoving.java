/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.commands;

import static edu.wpi.first.units.Units.Meters;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.andromedalib.util.AllianceFlipUtil;
import com.team6647.subsystems.SuperStructure;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;
import com.team6647.util.ShootingCalculatorUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootingWhileMoving extends Command {

  private AndromedaSwerve swerve;
  private SuperStructure superStructure;
  private Translation2d targetLocation;

  /** Creates a new ShootingWhileMoving. */
  public ShootingWhileMoving(AndromedaSwerve swerve, SuperStructure superStructure) {
    this.swerve = swerve;
    this.superStructure = superStructure;
    targetLocation = new Translation2d(Meters.of(0.06), Meters.of(5.54));

    targetLocation = AllianceFlipUtil.apply(targetLocation);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    Pose2d robotPose = swerve.getPose();
    /*
     * Translation2d speakerLoc = Speaker.centerSpeakerOpening.toTranslation2d();
     * Translation2d robotTranslation = robotPose.getTranslation();
     * 
     * Translation2d chassisToTarget = speakerLoc.minus(robotTranslation);
     * Rotation2d angle =
     * chassisToTarget.getAngle().rotateBy(Rotation2d.fromDegrees(180));
     */

    Translation2d speakerLoc = ShootingCalculatorUtil.calculateShootingWhileDriving(robotPose,
        swerve.getFieldRelativeChassisSpeeds());

    ShootingParameters parameters = ShootingCalculatorUtil.getShootingParameters(swerve.getPose(),
        speakerLoc);

    SuperStructure.updateShootingParameters(parameters);

    swerve.driveSetpoint(parameters.robotAngle(), false);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

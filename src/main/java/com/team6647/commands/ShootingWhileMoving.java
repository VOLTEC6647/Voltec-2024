/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.commands;

import static edu.wpi.first.units.Units.Meters;

import com.team6647.subsystems.SuperStructure;
import com.team6647.subsystems.drive.Drive;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;
import com.team6647.util.AllianceFlipUtil;
import com.team6647.util.ShootingCalculatorUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

@SuppressWarnings("unused")

public class ShootingWhileMoving extends Command {
  private Drive swerve;
  private SuperStructure superStructure;
  private Translation2d targetLocation;

  /** Creates a new ShootingWhileMoving. */
  public ShootingWhileMoving(Drive swerve, SuperStructure superStructure) {
    this.swerve = swerve;
    this.superStructure = superStructure;

    targetLocation = new Translation2d(Meters.of(0.06), Meters.of(5.54));

    targetLocation = AllianceFlipUtil.apply(targetLocation);
  }

  @Override
  public void initialize() {
    /*
     * swerve.setHeadingOverride(true);
     */ }

  @Override
  public void execute() {
    Pose2d robotPose = swerve.getPose();

    Translation2d speakerLoc = ShootingCalculatorUtil.calculateShootingWhileDriving(robotPose,
        swerve.getFieldRelativeChassisSpeeds());

    ShootingParameters parameters = ShootingCalculatorUtil.getShootingParameters(swerve.getPose(),
        speakerLoc);

    SuperStructure.updateShootingParameters(parameters);

    /*
     * swerve.setTargetHeading(parameters.robotAngle());
     */ }

  @Override
  public void end(boolean interrupted) {
    /*
     * swerve.setHeadingOverride(false);
     */ }

  @Override
  public boolean isFinished() {
    return false;
  }
}

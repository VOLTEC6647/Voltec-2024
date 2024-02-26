/**
 * Written by Juan Pablo Gutiérrez 
 */
package com.team6647.commands;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.andromedalib.util.AllianceFlipUtil;
import com.team6647.subsystems.SuperStructure;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.ShooterRollerSubsystem;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem.ShooterPivotState;
import com.team6647.subsystems.vision.VisionSubsystem;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.VisionConstants;
import com.team6647.util.Constants.FieldConstants.Speaker;
import com.team6647.util.Constants.RobotConstants.RollerState;
import com.team6647.util.ShootingCalculatorUtil;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootingStationary extends Command {

  private AndromedaSwerve swerve;
  private ShooterSubsystem flywheelSubsystem;
  private ShooterPivotSubsystem pivotSubsystem;
  private ShooterRollerSubsystem rollerSubsystem;
  private VisionSubsystem visionSubsystem;

  private ShootingParameters parameters;

  private int stageID;

  /** Creates a new ShootingStationary. */
  public ShootingStationary(AndromedaSwerve swerve, ShooterSubsystem flywheelSubsystem,
      ShooterPivotSubsystem pivotSubsystem, ShooterRollerSubsystem rollerSubsystem, VisionSubsystem visionSubsystem) {
    this.swerve = swerve;
    this.flywheelSubsystem = flywheelSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.rollerSubsystem = rollerSubsystem;
    this.visionSubsystem = visionSubsystem;

    addRequirements(swerve, flywheelSubsystem, pivotSubsystem, rollerSubsystem);

    stageID = AllianceFlipUtil.shouldFlip() ? VisionConstants.speakerRedCentgerTagID
        : VisionConstants.speakerBlueCenterTagID;

    visionSubsystem.changePipeline(VisionConstants.speakerPipelineNumber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.parameters = ShootingCalculatorUtil.getShootingParameters(swerve.getPose(),
        Speaker.centerSpeakerOpening.toTranslation2d());

    this.parameters = new ShootingParameters(parameters.robotAngle(), 154.0, 5000);

    SuperStructure.updateShootingParameters(parameters);
  }

  @Override
  public void execute() {
/*     if (visionSubsystem.hasTargetID(stageID)) {
      // kP (constant of proportionality)
      // this is a hand-tuned number that determines the aggressiveness of our
      // proportional control loop
      // if it is too high, the robot will oscillate around.
      // if it is too low, the robot will never reach its target
      // if the robot never turns in the correct direction, kP should be inverted.
      double kP = .035;

      // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
      // rightmost edge of
      // your limelight 3 feed, tx should return roughly 31 degrees.
      double targetingAngularVelocity = visionSubsystem.getTX() * kP;

      // convert to radians per second for our drive method
      targetingAngularVelocity *= DriveConstants.maxAngularVelocity;

      // invert since tx is positive when the target is to the right of the crosshair
      targetingAngularVelocity *= -1.0;

      swerve.drive(new Translation2d(), targetingAngularVelocity, true);
    } else {
    } */
    
    swerve.driveSetpoint(parameters.robotAngle(), true);
    
    flywheelSubsystem.changeFlywheelState(FlywheelState.SHOOTING);
    pivotSubsystem.setShooterPivotState(ShooterPivotState.SHOOTING);

    if (flywheelSubsystem.topInTolerance() && flywheelSubsystem.bottomInTolerance()) {
      rollerSubsystem.changeRollerState(RollerState.INTAKING);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    visionSubsystem.changePipeline(VisionConstants.odometryPipelineNumber);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

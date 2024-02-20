/**
 * Written by Juan Pablo Gutiérrez 
 */
package com.team6647.commands;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.team6647.subsystems.SuperStructure;
import com.team6647.subsystems.SuperStructure.SuperStructureState;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.ShooterRollerSubsystem;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem.ShooterPivotState;
import com.team6647.util.Constants.FieldConstants.Speaker;
import com.team6647.util.Constants.RobotConstants.RollerState;
import com.team6647.util.ShootingCalculatorUtil;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootingStationary extends Command {

  private AndromedaSwerve swerve;
  private ShooterSubsystem flywheelSubsystem;
  private ShooterPivotSubsystem pivotSubsystem;
  private ShooterRollerSubsystem rollerSubsystem;
  private SuperStructure superStructure;

  private ShootingParameters parameters;

  /** Creates a new ShootingStationary. */
  public ShootingStationary(AndromedaSwerve swerve, ShooterSubsystem flywheelSubsystem,
      ShooterPivotSubsystem pivotSubsystem, ShooterRollerSubsystem rollerSubsystem, SuperStructure superStructure) {
    this.swerve = swerve;
    this.flywheelSubsystem = flywheelSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.rollerSubsystem = rollerSubsystem;

    addRequirements(swerve, flywheelSubsystem, pivotSubsystem, rollerSubsystem);

    this.superStructure = superStructure;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.parameters = ShootingCalculatorUtil.getShootingParameters(swerve.getPose(),
        Speaker.centerSpeakerOpening.toTranslation2d());

    superStructure.updateShootingParameters(parameters);
  }

  @Override
  public void execute() {
    swerve.driveSetpoint(parameters.robotAngle());
    flywheelSubsystem.changeFlywheelState(FlywheelState.SHOOTING);
    pivotSubsystem.setShooterPivotState(ShooterPivotState.SHOOTING);

    if (flywheelSubsystem.topInTolerance() && flywheelSubsystem.bottomInTolerance()) {
      rollerSubsystem.changeRollerState(RollerState.INTAKING);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

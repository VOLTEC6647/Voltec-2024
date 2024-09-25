/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 11 04 2023
 * 
 * Moves de pivot up or down until it is safe to enable
 */
package com.team6647.commands;

import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem.ShooterPivotState;

import edu.wpi.first.wpilibj2.command.Command;

public class PivotEnable extends Command {

  private ShooterPivotSubsystem shooterPivotSubsystem;
  private Direction direction;

  /** Creates a new PivotEnable. */
  public PivotEnable(
      ShooterPivotSubsystem shooterPivotSubsystem, Direction direction) {
    this.shooterPivotSubsystem = shooterPivotSubsystem;
    this.direction = direction;

    //addRequirements(shooterPivotSubsystem);
  }

  private enum Direction {
    UP,
    DOWN;
  }

  @Override
  public void initialize() {
    if (!(shooterPivotSubsystem.getMState() == ShooterPivotState.EMERGENCY_DISABLED)) {
      end(true);
    }
  }

  @Override
  public void execute() {
    if (direction == Direction.UP) {
      shooterPivotSubsystem.runPivotVolts(0.2);
    } else {
      shooterPivotSubsystem.runPivotVolts(-0.2);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      shooterPivotSubsystem.runPivotVolts(0);
      shooterPivotSubsystem.setShooterPivotState(ShooterPivotState.HOMED);
      shooterPivotSubsystem.enablePivot();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 19 02 2024
 */
package com.team6647.commands;

import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem.ShooterPivotState;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterPivotTarget extends Command {

  private ShooterPivotSubsystem shooterPivotSubsystem;
  private ShooterPivotState pivotState;

  public ShooterPivotTarget(ShooterPivotSubsystem shooterPivotSubsystem, ShooterPivotState shooterPivotState) {
    this.shooterPivotSubsystem = shooterPivotSubsystem;
    this.pivotState = shooterPivotState;

    addRequirements(shooterPivotSubsystem);
  }

  @Override
  public void initialize() {
    shooterPivotSubsystem.setMState(pivotState);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return shooterPivotSubsystem.inTolerance();
  }
}

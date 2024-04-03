/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 19 02 2024
 */
package com.team6647.commands;

import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem;
import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem.ShooterFeederState;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterRollerStartEnd extends Command {
  private ShooterRollerSubsystem shooterRollerSubsystem;
  private ShooterFeederState initialRollerState;
  private ShooterFeederState finalRollerState;

  public ShooterRollerStartEnd(ShooterRollerSubsystem shooterRollerSubsystem, ShooterFeederState initialRollerState,
      ShooterFeederState finalRollerState) {
    this.shooterRollerSubsystem = shooterRollerSubsystem;
    this.initialRollerState = initialRollerState;
    this.finalRollerState = finalRollerState;

    addRequirements(shooterRollerSubsystem);
  }

  @Override
  public void initialize() {
    shooterRollerSubsystem.setMRollerState(initialRollerState);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    shooterRollerSubsystem.setMRollerState(finalRollerState);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

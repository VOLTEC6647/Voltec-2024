/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 19 02 2024
 */
package com.team6647.commands;

import com.team6647.subsystems.shooter.ShooterRollerSubsystem;
import com.team6647.util.Constants.RobotConstants.RollerState;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterRollerStartEnd extends Command {
  private ShooterRollerSubsystem shooterRollerSubsystem;
  private RollerState initialRollerState;
  private RollerState finalRollerState;

  public ShooterRollerStartEnd(ShooterRollerSubsystem shooterRollerSubsystem, RollerState initialRollerState,
      RollerState finalRollerState) {
    this.shooterRollerSubsystem = shooterRollerSubsystem;
    this.initialRollerState = initialRollerState;
    this.finalRollerState = finalRollerState;

    addRequirements(shooterRollerSubsystem);
  }

  @Override
  public void initialize() {
    shooterRollerSubsystem.changeRollerState(initialRollerState);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    shooterRollerSubsystem.changeRollerState(finalRollerState);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

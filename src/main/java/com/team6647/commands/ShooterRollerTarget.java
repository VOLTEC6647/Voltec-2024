/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 19 02 2024
 */
package com.team6647.commands;

import com.team6647.subsystems.shooter.ShooterRollerSubsystem;
import com.team6647.util.Constants.RobotConstants.RollerState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterRollerTarget extends InstantCommand {
  private ShooterRollerSubsystem shooterRollerSubsystem;
  private RollerState rollerState;

  public ShooterRollerTarget(ShooterRollerSubsystem shooterRollerSubsystem, RollerState rollerState) {
    this.shooterRollerSubsystem = shooterRollerSubsystem;
    this.rollerState = rollerState;

    addRequirements(shooterRollerSubsystem);
  }

  @Override
  public void initialize() {
    shooterRollerSubsystem.changeRollerState(rollerState);
  }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 19 02 2024
 */
package com.team6647.commands;

import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FlywheelTarget extends InstantCommand {
  private ShooterSubsystem shooterSubsystem;
  private FlywheelState flywheelState;

  public FlywheelTarget(ShooterSubsystem shooterSubsystem, FlywheelState flywheelState) {
    this.shooterSubsystem = shooterSubsystem;
    this.flywheelState = flywheelState;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.changeFlywheelState(flywheelState);
  }
}

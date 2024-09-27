/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 19 02 2024
 */

package com.team6647.commands;

import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;

import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelStartEnd extends Command {
  private ShooterSubsystem flywheelSubsystem;
  private FlywheelState initialFlywheelState;
  private FlywheelState finalFlywheelState;

  public FlywheelStartEnd(ShooterSubsystem shooterSubsystem, FlywheelState initialFlywheelState,
      FlywheelState finalFlywheelState) {
    this.flywheelSubsystem = shooterSubsystem;
    this.initialFlywheelState = initialFlywheelState;
    this.finalFlywheelState = finalFlywheelState;

    //addRequirements(flywheelSubsystem);
  }

  @Override
  public void initialize() {
    flywheelSubsystem.setFlywheelState(initialFlywheelState);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    flywheelSubsystem.setFlywheelState(finalFlywheelState);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

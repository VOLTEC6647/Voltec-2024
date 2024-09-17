/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.commands;

import com.team6647.subsystems.flywheel.ShooterIOInputsAutoLogged;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterIO.ShooterIOInputs;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;

import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelTarget extends Command {
  private ShooterSubsystem shooterSubsystem;
  private FlywheelState flywheelState;

  /** Creates a new FlywheelTargetFinish. */
  public FlywheelTarget(ShooterSubsystem shooterSubsystem, FlywheelState flywheelState) {
    this.shooterSubsystem = shooterSubsystem;
    this.flywheelState = flywheelState;

    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setFlywheelState(flywheelState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSubsystem.ready();
  }
}

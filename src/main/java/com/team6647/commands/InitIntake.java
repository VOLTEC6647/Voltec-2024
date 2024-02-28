/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 26 02 2024
 */
package com.team6647.commands;

import com.team6647.subsystems.intake.IntakePivotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class InitIntake extends Command {

  private IntakePivotSubsystem intakePivotSubsystem;

  /** Creates a new InitIntake. */
  public InitIntake(IntakePivotSubsystem intakePivotSubsystem) {
    this.intakePivotSubsystem = intakePivotSubsystem;

    addRequirements(intakePivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakePivotSubsystem.setPushingPercentage(-0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakePivotSubsystem.setPushingPercentage(0);
    intakePivotSubsystem.setPushingPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakePivotSubsystem.getPushingPressed();
  }
}

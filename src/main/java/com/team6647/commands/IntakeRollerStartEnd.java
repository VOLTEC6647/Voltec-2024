/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 19 02 2024
 */
package com.team6647.commands;

import com.team6647.subsystems.intake.roller.IntakeSubsystem;
import com.team6647.subsystems.intake.roller.IntakeSubsystem.IntakeRollerState;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeRollerStartEnd extends Command {
  private IntakeSubsystem intakeSubsystem;
  private IntakeRollerState startState;
  private IntakeRollerState endState;

  public IntakeRollerStartEnd(IntakeSubsystem intakeSubsystem, IntakeRollerState startState, IntakeRollerState endState) {
    this.intakeSubsystem = intakeSubsystem;
    this.startState = startState;
    this.endState = endState;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.setMState(startState);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMState(endState);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

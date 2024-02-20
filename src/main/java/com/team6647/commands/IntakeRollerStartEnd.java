/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 19 02 2024
 */
package com.team6647.commands;

import com.team6647.subsystems.intake.IntakeSubsystem;
import com.team6647.util.Constants.RobotConstants.RollerState;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeRollerStartEnd extends Command {
  private IntakeSubsystem intakeSubsystem;
  private RollerState startState;
  private RollerState endState;

  public IntakeRollerStartEnd(IntakeSubsystem intakeSubsystem, RollerState startState, RollerState endState) {
    this.intakeSubsystem = intakeSubsystem;
    this.startState = startState;
    this.endState = endState;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.changeRollerState(startState);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.changeRollerState(endState);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

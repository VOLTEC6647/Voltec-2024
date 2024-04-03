/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 19 02 2024
 */

package com.team6647.commands;

import com.team6647.subsystems.intake.roller.IntakeSubsystem;
import com.team6647.subsystems.intake.roller.IntakeSubsystem.IntakeRollerState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeRollerTarget extends InstantCommand {
  private IntakeSubsystem intakeSubsystem;
  private IntakeRollerState rollerState;

  public IntakeRollerTarget(IntakeSubsystem intakeSubsystem, IntakeRollerState rollerState) {
    this.intakeSubsystem = intakeSubsystem;
    this.rollerState = rollerState;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.setMState(rollerState);
  }
}

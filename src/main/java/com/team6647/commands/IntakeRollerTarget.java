/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 19 02 2024
 */

package com.team6647.commands;

import com.team6647.subsystems.intake.IntakeSubsystem;
import com.team6647.util.Constants.RobotConstants.RollerState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeRollerTarget extends InstantCommand {
  private IntakeSubsystem intakeSubsystem;
  private RollerState rollerState;

  public IntakeRollerTarget(IntakeSubsystem intakeSubsystem, RollerState rollerState) {
    this.intakeSubsystem = intakeSubsystem;
    this.rollerState = rollerState;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.changeRollerState(rollerState);
  }
}

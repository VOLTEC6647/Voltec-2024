/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 19 02 2024
 */

package com.team6647.commands;

import com.team6647.subsystems.intake.IntakePivotSubsystem;
import com.team6647.subsystems.intake.IntakePivotSubsystem.IntakePivotState;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * 
 * @deprecated Use {@link IntakeExtend} or {@link IntakeHome} instead
 */
public class IntakePivotTarget extends Command {

  private IntakePivotSubsystem intakePivotSubsystem;
  private IntakePivotState intakePivotState;

  public IntakePivotTarget(IntakePivotSubsystem intakePivotSubsystem, IntakePivotState intakePivotState) {
    this.intakePivotSubsystem = intakePivotSubsystem;
    this.intakePivotState = intakePivotState;

    addRequirements(intakePivotSubsystem);
  }

  @Override
  public void initialize() {
    intakePivotSubsystem.changeIntakePivotState(intakePivotState);
  }

  @Override
  public void execute() {

  }
}

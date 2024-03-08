/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 27 02 2024
 */
package com.team6647.commands;

import com.team6647.RobotContainer;
import com.team6647.subsystems.intake.pivot.IntakePivotSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class IntakeExtend extends SequentialCommandGroup {

  private IntakePivotSubsystem intakePivotSubsystem = RobotContainer.intakePivotSubsystem;

  /** Creates a new IntakeExtend. */
  public IntakeExtend() {
    addCommands(
        new IntakePush(intakePivotSubsystem),
        new StartEndCommand(() -> intakePivotSubsystem.setIntakeVoltage(0.8),
            () -> intakePivotSubsystem.setIntakeVoltage(0), intakePivotSubsystem).withTimeout(0.5));
  }
}

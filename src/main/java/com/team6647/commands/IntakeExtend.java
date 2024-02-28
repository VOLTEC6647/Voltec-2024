/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 27 02 2024
 */
package com.team6647.commands;

import com.team6647.RobotContainer;
import com.team6647.subsystems.intake.IntakePivotSubsystem;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeExtend extends SequentialCommandGroup {

  private IntakePivotSubsystem intakePivotSubsystem = RobotContainer.intakePivotSubsystem;

  /** Creates a new IntakeExtend. */
  public IntakeExtend() {
    addCommands(
        new IntakePush(intakePivotSubsystem).withTimeout(0.5),
        new RunCommand(() -> intakePivotSubsystem.setIntakeVoltage(0.2), intakePivotSubsystem).withTimeout(0.5));
  }
}

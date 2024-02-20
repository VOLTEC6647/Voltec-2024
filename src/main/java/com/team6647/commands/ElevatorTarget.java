/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 19 02 2024
 */

package com.team6647.commands;

import com.team6647.subsystems.elevator.ElevatorSubsystem;
import com.team6647.subsystems.elevator.ElevatorSubsystem.ElevatorState;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorTarget extends Command {

  private ElevatorSubsystem elevatorSubsystem;
  private ElevatorState elevatorState;

  public ElevatorTarget(ElevatorSubsystem elevatorSubsystem, ElevatorState elevatorState) {
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.changeElevatorState(elevatorState);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.inTolerance();
  }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 22 02 2024
 */
package com.team6647.commands;

import com.team6647.RobotContainer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeTriggerCommand extends Command {

  private Timer durationTimer = new Timer();

  /** Creates a new IntakeTriggerCommand. */
  public IntakeTriggerCommand() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    durationTimer.restart();
    durationTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Timer", durationTimer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.intakeSubsystem.getAmps() > 5) && (durationTimer.get() > 0.2);
  }
}

/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.commands;

import com.team6647.subsystems.vision.VisionSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class LimelightBlinkEffect extends Command {
  private VisionSubsystem visionSubsystem;

  double startTime = 0.0;

  /** Creates a new LimelightBlinkEffect. */
  public LimelightBlinkEffect(VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem = visionSubsystem;

    addRequirements(visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionSubsystem.setLimelightMode(2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    visionSubsystem.setLimelightMode(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > 3;
  }
}

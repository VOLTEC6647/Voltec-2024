/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 20 02 2024
 */
package com.team6647.commands;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.team6647.subsystems.SuperStructure;
import com.team6647.subsystems.SuperStructure.SuperStructureState;
import com.team6647.subsystems.intake.IntakePivotSubsystem;
import com.team6647.subsystems.intake.IntakeSubsystem;
import com.team6647.subsystems.neural.NeuralVisionSubsystem;
import com.team6647.util.Constants.DriveConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionIntakeAlign extends Command {
  private NeuralVisionSubsystem neuralVisionSubsystem;
  private AndromedaSwerve andromedaSwerve;

  /** Creates a new IntakeAlign. */
  public VisionIntakeAlign(NeuralVisionSubsystem neuralVisionSubsystem, AndromedaSwerve andromedaSwerve) {
    this.neuralVisionSubsystem = neuralVisionSubsystem;
    this.andromedaSwerve = andromedaSwerve;

    addRequirements(neuralVisionSubsystem, andromedaSwerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double kp = 0.004;
    double kpRange = 0.05;

    double targetingXVel = neuralVisionSubsystem.getTX() * kp;
    double targetingYVel = neuralVisionSubsystem.getTY() * kpRange;
    targetingXVel *= DriveConstants.maxAngularVelocity;

    targetingXVel *= -1;
    targetingYVel *= -1;

    andromedaSwerve.drive(new Translation2d(targetingYVel, 0), targetingXVel, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !neuralVisionSubsystem.hasTarget() || neuralVisionSubsystem.getTY() < -12;
  }
}

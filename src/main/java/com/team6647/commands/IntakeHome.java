/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.commands;

import org.littletonrobotics.junction.Logger;

import com.team6647.subsystems.intake.IntakePivotSubsystem;
import com.team6647.subsystems.intake.IntakePivotSubsystem.IntakePivotState;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeHome extends Command {

  private IntakePivotSubsystem intakePivotSubsystem;

  private PIDController mController = new PIDController(IntakeConstants.homedKp,
      IntakeConstants.homedKi, IntakeConstants.homedKd);

  /** Creates a new IntakeHome. */
  public IntakeHome(IntakePivotSubsystem intakePivotSubsystem) {
    this.intakePivotSubsystem = intakePivotSubsystem;

    addRequirements(intakePivotSubsystem);

    mController.setTolerance(IntakeConstants.homedTolerance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakePivotSubsystem.changeIntakePivotState(IntakePivotState.HOMED);
    intakePivotSubsystem.setPushingReference(IntakeConstants.pushingHomedPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = mController.calculate(intakePivotSubsystem.intakePosition(),
        IntakeConstants.intakeExtendedPosition);

    Logger.recordOutput("Intake/Pivot/output", output);

    intakePivotSubsystem.setIntakeVoltage(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mController.atSetpoint();
  }
}

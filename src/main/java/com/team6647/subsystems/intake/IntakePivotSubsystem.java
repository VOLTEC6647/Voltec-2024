/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 26 01 2024
 */
package com.team6647.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.math.Functions;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {

  private static IntakePivotSubsystem instance;

  @AutoLogOutput(key = "Intake/Pivot/State")
  private IntakePivotState mState = IntakePivotState.HOMED;

  private IntakePivotIO io;
  private IntakePivoIOInputsAutoLogged inputs = new IntakePivoIOInputsAutoLogged();

  private PIDController mExtendedPivotController = new PIDController(IntakeConstants.extendedKp,
      IntakeConstants.extendedKi, IntakeConstants.extendedKd);

  private PIDController mHomedPivotController = new PIDController(IntakeConstants.homedKp,
      IntakeConstants.homedKi, IntakeConstants.homedKd);

  @AutoLogOutput(key = "Intake/Pivot/Setpoint")
  private double setpoint = IntakeConstants.intakeHomedPosition;

  /** Creates a new IntakePivotSubsystem. */
  private IntakePivotSubsystem(IntakePivotIO io) {
    this.io = io;

    mExtendedPivotController.setTolerance(IntakeConstants.intakePivotPositionTolerance);

    resetPID();
  }

  public static IntakePivotSubsystem getInstance(IntakePivotIO io) {
    if (instance == null) {
      instance = new IntakePivotSubsystem(io);
    }
    return instance;
  }

  public enum IntakePivotState {
    HOMED,
    EXTENDED
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Pivot", inputs);

    computePID(mState == IntakePivotState.HOMED);
  }

  public void changeIntakePivotState(IntakePivotState intakePivotState) {
    switch (intakePivotState) {
      case HOMED:
        mState = IntakePivotState.HOMED;
        changeSetpoint(IntakeConstants.intakeHomedPosition);
        break;
      case EXTENDED:
        mState = IntakePivotState.EXTENDED;
        changeSetpoint(IntakeConstants.intakeExtendedPosition);
        break;
      default:
        break;
    }
  }

  private void changeSetpoint(double newSetpoint) {
    if (newSetpoint < IntakeConstants.maxIntakePivotPosition || newSetpoint > IntakeConstants.minIntakePivotPosition) {
      newSetpoint = Functions.clamp(newSetpoint, IntakeConstants.minIntakePivotPosition,
          IntakeConstants.maxIntakePivotPosition);
    }

    setpoint = newSetpoint;
  }

  private void computePID(boolean equalOutput) {
    double output = equalOutput ? mHomedPivotController.calculate(inputs.intakePivotAbsoluteEncoderPosition, setpoint)
        : mExtendedPivotController.calculate(inputs.intakePivotAbsoluteEncoderPosition, setpoint);

    double feedforwardValue = equalOutput ? output : -output * 2.1;

    output = output * 12;
    feedforwardValue = feedforwardValue * 12;

    Logger.recordOutput("Intake/Pivot/output", output);
    Logger.recordOutput("Intake/Pivot/feedforward", feedforwardValue);

    io.setIntakeVoltage(feedforwardValue, output);
  }

  @AutoLogOutput(key = "Intake/Pivot/InTolerance")
  public boolean inTolerance() {
    return mExtendedPivotController.atSetpoint();
  }

  public void resetPID() {

    // mExtendedPivotController.reset(inputs.intakePivotAbsoluteEncoderPosition);
    // mHomedPivotController.reset(inputs.intakePivotAbsoluteEncoderPosition);
  }

}

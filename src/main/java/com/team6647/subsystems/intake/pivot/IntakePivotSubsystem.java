/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 26 01 2024
 */
package com.team6647.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.math.Functions;
import com.andromedalib.util.Alert;
import com.andromedalib.util.Alert.AlertType;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Setter;

public class IntakePivotSubsystem extends SubsystemBase {

  private static IntakePivotSubsystem instance;

  @Setter
  @AutoLogOutput(key = "Intake/Pivot/State")
  public IntakePivotState mState = IntakePivotState.HOMED;

  private IntakePivotIO io;
  private IntakePivoIOInputsAutoLogged inputs = new IntakePivoIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/Pivot/Setpoint")
  private double setpoint = IntakeConstants.intakeHomedPosition;

  @AutoLogOutput(key = "Intake/Pivot/PushingSetpoint")
  private double pushingSetpoint = IntakeConstants.pushingAcutatingPosition;

  private Alert pivotEncoderAlert = new Alert("Intake Pivot Encoder disconnected", AlertType.WARNING);

  /** Creates a new IntakePivotSubsystem. */
  private IntakePivotSubsystem(IntakePivotIO io) {
    this.io = io;
  }

  public static IntakePivotSubsystem getInstance(IntakePivotIO io) {
    if (instance == null) {
      instance = new IntakePivotSubsystem(io);
    }
    return instance;
  }

  public enum IntakePivotState {
    HOMED,
    EXTENDED,
    EMERGENCY_DISABLED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Pivot", inputs);

    if (getCurrentCommand() != null) {
      Logger.recordOutput("Intake/Pivot/CurrentCommand", getCurrentCommand().getName());
    } else {
      Logger.recordOutput("Intake/Pivot/CurrentCommand", "");
    }

    pivotEncoderAlert.set(inputs.intakePivotAbsoluteEncoderPosition == 0);

    if (inputs.intakePivotAbsoluteEncoderPosition == 0) {
      mState = IntakePivotState.EMERGENCY_DISABLED;
      io.disableIntake();
    }

    switch (mState) {
      case HOMED:
        changeSetpoint(IntakeConstants.intakeHomedPosition);
        break;
      case EXTENDED:
        changeSetpoint(IntakeConstants.intakeExtendedPosition);
        break;
      case EMERGENCY_DISABLED:
        changeSetpoint(inputs.intakePivotAbsoluteEncoderPosition);
        break;
    }
  }

  // Redudant, but will stay for logging purposes
  private void changeSetpoint(double newSetpoint) {
    if (newSetpoint > IntakeConstants.maxIntakePivotPosition || newSetpoint < IntakeConstants.minIntakePivotPosition) {
      newSetpoint = Functions.clamp(newSetpoint, IntakeConstants.minIntakePivotPosition,
          IntakeConstants.maxIntakePivotPosition);
    }

    setpoint = newSetpoint;
  }

  public void setPushingReference(double reference) {
    io.setPushingReference(reference);
  }

  public void setIntakeVoltage(double volts) {
    io.setIntakeVoltage(volts);
  }

  public void setPushingPercentage(double percentage) {
    io.setPushingPercentage(percentage);
  }

  public void setPushingPosition(double position) {
    io.setPushingPosition(position);
  }

  @AutoLogOutput(key="Intake/Pivot/PushingInTolerance")
  public boolean pushingInTolerance() {
    return Math.abs(inputs.intakePushingMotorPosition - pushingSetpoint) < 1;
  }

  public double intakePosition() {
    return inputs.intakePivotAbsoluteEncoderPosition;
  }

  @AutoLogOutput(key = "Intake/Pivot/IntakeInTolerance")
  public boolean inTolerance() {
    return Math.abs(inputs.intakePivotAbsoluteEncoderPosition - setpoint) < IntakeConstants.homedTolerance;
  }

  public boolean getPushingPressed() {
    return inputs.pushingLimitSwitchPressed;
  }

  public boolean emergencyDisabled() {
    return mState == IntakePivotState.EMERGENCY_DISABLED;
  }

}

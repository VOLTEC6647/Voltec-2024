/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 26 01 2024
 */
package com.team6647.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.math.Functions;
import com.team6647.util.LoggedTunableNumber;
import com.team6647.util.Constants.IntakeConstants;
import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {

  private static IntakePivotSubsystem instance;

  @AutoLogOutput(key = "Intake/Pivot/State")
  private IntakePivotState mState = IntakePivotState.HOMED;

  private IntakePivotIO io;
  private IntakePivoIOInputsAutoLogged inputs = new IntakePivoIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/Pivot/Setpoint")
  private double setpoint = IntakeConstants.intakeHomedPosition;

  @AutoLogOutput(key = "Intake/Pivot/PushingSetpoint")
  private double pushingSetpoint = IntakeConstants.pushingAcutatingPosition;

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
    EMERGENCY_DISABLED
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Pivot", inputs);

    if (inputs.intakePivotAbsoluteEncoderPosition == 0 || mState == IntakePivotState.EMERGENCY_DISABLED) {
      mState = IntakePivotState.EMERGENCY_DISABLED;
      DriverStation.reportError("[" + getName() + "] Absolute Encoder position is not in range. Emergency disabled",
          true);
    }

    if (getCurrentCommand() != null) {
      Logger.recordOutput("Intake/Pivot/CurrentCommand", getCurrentCommand().getName());
    } else {
      Logger.recordOutput("Intake/Pivot/CurrentCommand", "");
    }

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
      case EMERGENCY_DISABLED:
        mState = IntakePivotState.EMERGENCY_DISABLED;
        break;
    }
  }

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

  public boolean pushingInTolerance() {
    return Math.abs(inputs.intakePivotLeftMotorPosition - pushingSetpoint) < 1;
  }

  public double intakePosition(){
    return inputs.intakePivotAbsoluteEncoderPosition;
  }

  @AutoLogOutput(key = "Intake/Pivot/InTolerance")
  public boolean inTolerance() {
    return Math.abs(inputs.intakePivotAbsoluteEncoderPosition - setpoint) < IntakeConstants.homedTolerance;
  }

}

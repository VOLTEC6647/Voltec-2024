/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.math.Functions;
import com.team6647.util.LoggedTunableNumber;
import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivotSubsystem extends SubsystemBase {

  private static ShooterPivotSubsystem instance;

  @AutoLogOutput(key = "Shooter/Pivot/State")
  private static ShooterPivotState mState = ShooterPivotState.INDEXING;

  private ShooterPivotIO io;
  private ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

  // @AutoLogOutput(key = "Shooter/Pivot/Setpoint")
  // private double setpoint = ShooterConstants.pivotIndexingPosition;

  private LoggedTunableNumber pivotKp = new LoggedTunableNumber("Shooter/Pivot/kp", ShooterConstants.pivotKp);
  private LoggedTunableNumber pivotKi = new LoggedTunableNumber("Shooter/Pivot/ki", ShooterConstants.pivotKi);
  private LoggedTunableNumber pivotKd = new LoggedTunableNumber("Shooter/Pivot/kd", ShooterConstants.pivotKd);
  private LoggedTunableNumber pivotKf = new LoggedTunableNumber("Shooter/Pivot/kf", ShooterConstants.pivotKf);

  private LoggedTunableNumber pivotSetpoint = new LoggedTunableNumber("Shooter/Pivot/Setpoint",
      ShooterConstants.pivotIndexingPosition);

  /** Creates a new ShooterPivotSubsystem. */
  private ShooterPivotSubsystem(
      ShooterPivotIO io) {
    this.io = io;
  }

  public static ShooterPivotSubsystem getInstance(ShooterPivotIO io) {
    if (instance == null) {
      instance = new ShooterPivotSubsystem(io);
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Pivot", inputs);

    LoggedTunableNumber.ifChanged(hashCode(), pid -> {
      io.setPIDF(pid[0], pid[1], pid[2], pid[3]);

      io.setShooterReference(pid[4]);
    }, pivotKp, pivotKi, pivotKd, pivotKf, pivotSetpoint);
  }

  public enum ShooterPivotState {
    HOMED,
    SHOOTING,
    INDEXING,
  }

  public void setShooterPivotState(ShooterPivotState state) {
    switch (state) {
      case HOMED:
        mState = ShooterPivotState.HOMED;
        changeSetpoint(ShooterConstants.pivotHomedPosition);
        break;
      case SHOOTING:
        mState = ShooterPivotState.SHOOTING;
        break;
      case INDEXING:
        mState = ShooterPivotState.INDEXING;
        changeSetpoint(ShooterConstants.pivotIndexingPosition);
      default:
        break;
    }
  }

  private void changeSetpoint(double newSetpoint) {
    if (newSetpoint > ShooterConstants.pivotMaxPosition || newSetpoint < ShooterConstants.pivotMinPosition) {
      newSetpoint = Functions.clamp(newSetpoint, ShooterConstants.pivotMinPosition,
          ShooterConstants.pivotMaxPosition);
    }

    // setpoint = newSetpoint;
    // io.setShooterReference(setpoint);
  }

  public boolean inTolerance() {
    return inputs.inTolerance;
  }

}

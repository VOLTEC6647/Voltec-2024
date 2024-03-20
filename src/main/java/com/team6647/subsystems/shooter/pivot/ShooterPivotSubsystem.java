/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter.pivot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.math.Functions;
import com.team6647.subsystems.leds.LEDSubsystem;
import com.team6647.util.Alert;
import com.team6647.util.LoggedTunableNumber;
import com.team6647.util.Alert.AlertType;
import com.team6647.util.Constants.ShooterConstants;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.RequiredArgsConstructor;

public class ShooterPivotSubsystem extends SubsystemBase {

  private static ShooterPivotSubsystem instance;

  @AutoLogOutput(key = "Shooter/Pivot/State")
  public ShooterPivotState mState = ShooterPivotState.HOMED;

  private ShooterPivotIO io;
  private ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

  private LoggedTunableNumber pivotKp = new LoggedTunableNumber("Shooter/Pivot/kp", ShooterConstants.pivotKp);
  private LoggedTunableNumber pivotKi = new LoggedTunableNumber("Shooter/Pivot/ki", ShooterConstants.pivotKi);
  private LoggedTunableNumber pivotKd = new LoggedTunableNumber("Shooter/Pivot/kd", ShooterConstants.pivotKd);
  private LoggedTunableNumber pivotKf = new LoggedTunableNumber("Shooter/Pivot/kf", ShooterConstants.pivotKf);

  private LoggedTunableNumber pivotSetpoint = new LoggedTunableNumber("Shooter/Pivot/Setpoint",
      ShooterConstants.pivotHomedPosition);

  private static ShootingParameters currentParameters;

  @AutoLogOutput(key = "Shooter/Pivot/Emergency Disable")
  private boolean emergencyDisable = false;

  @AutoLogOutput(key = "Shooter/Pivot/Setpoint")
  private double setpoint = ShooterConstants.pivotHomedPosition;

  private Alert pivotEncoderAlert = new Alert("Shooter Pivot Encoder disconnected", AlertType.WARNING);

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

    pivotEncoderAlert.set(inputs.shooterAbsoluteEncoderPosition == 0);

    if (inputs.shooterAbsoluteEncoderPosition == 0) {
      setShooterPivotState(ShooterPivotState.EMERGENCY_DISABLED);
      io.disablePivot();
    }

    if (mState == ShooterPivotState.EMERGENCY_DISABLED) {
      io.setShooterReference(inputs.pivotMotorPosition);
      io.disablePivot();
    }

    LoggedTunableNumber.ifChanged(hashCode(), pid -> {
      io.setPIDF(pid[0], pid[1], pid[2], pid[3]);

      changeSetpoint(pid[4]);
    }, pivotKp, pivotKi, pivotKd, pivotKf, pivotSetpoint);

    if (mState != ShooterPivotState.HOMED) {
      LEDSubsystem.getInstance().pivotHomed = false;
    } else {
      LEDSubsystem.getInstance().pivotHomed = true;
    }
  }

  @RequiredArgsConstructor
  public enum ShooterPivotState {
    HOMED(ShooterConstants.pivotHomedPosition),
    SHOOTING(-1),
    AMP(ShooterConstants.pivotAmpPosition),
    INDEXING(ShooterConstants.pivotIndexingPosition),
    CLIMBING(ShooterConstants.pivotClimbPosition),
    EMERGENCY_DISABLED(-1),
    CUSTOM(-1);

    private final double setpoint;
  }

  public void setShooterPivotState(ShooterPivotState state) {
    mState = state;

    switch (state) {
      case HOMED:
        changeSetpoint(state.setpoint);
        break;
      case SHOOTING:
        changeSetpoint(currentParameters.pivotAngle());
        break;
      case AMP:
        changeSetpoint(state.setpoint);
        break;
      case INDEXING:
        changeSetpoint(state.setpoint);
        break;
      case CLIMBING:
        changeSetpoint(state.setpoint);
        break;
      case EMERGENCY_DISABLED:
        changeSetpoint(inputs.pivotMotorPosition);
        io.disablePivot();
        break;
      case CUSTOM:
        changeSetpoint(setpoint);
        break;
    }
  }

  /**
   * Public security measure for arbitrarily changing the setpoint
   * 
   * @param newSetpoint The new setpoint
   */
  public void changeSetpoint(double newSetpoint) {
    if (newSetpoint > ShooterConstants.pivotMaxPosition || newSetpoint < ShooterConstants.pivotMinPosition) {
      newSetpoint = Functions.clamp(newSetpoint, ShooterConstants.pivotMinPosition,
          ShooterConstants.pivotMaxPosition);
    }

    setpoint = newSetpoint;

    io.setShooterReference(newSetpoint);
  }

  public boolean inTolerance() {
    return inputs.inTolerance;
  }

  public static void updateShootingParameters(ShootingParameters newParameters) {
    currentParameters = newParameters;
  }
}

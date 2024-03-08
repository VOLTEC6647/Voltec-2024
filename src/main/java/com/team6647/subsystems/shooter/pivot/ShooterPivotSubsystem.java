/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter.pivot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.math.Functions;
import com.team6647.util.LoggedTunableNumber;
import com.team6647.util.Constants.ShooterConstants;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ShooterPivotSubsystem extends SubsystemBase {

  private static ShooterPivotSubsystem instance;

  @Setter
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

    if (inputs.shooterAbsoluteEncoderPosition == 0) {
      DriverStation.reportError("[" + getName() + "] Absolute Encoder position is not in range. Emergency disabled",
          true);
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

    if (mState != ShooterPivotState.EMERGENCY_DISABLED || mState != ShooterPivotState.SHOOTING
        || mState != ShooterPivotState.CUSTOM) {
      setpoint = mState.setpoint;

      io.setShooterReference(mState.setpoint);
    }

    if (mState == ShooterPivotState.SHOOTING) {
      io.setShooterReference(currentParameters.pivotAngle());
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

    System.out.println("Setpoint changed to " + newSetpoint);

    //setMState(mState);

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

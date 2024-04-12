/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter.pivot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.math.Functions;
import com.andromedalib.util.Alert;
import com.andromedalib.util.Alert.AlertType;
import com.team6647.subsystems.leds.LEDSubsystem;
import com.team6647.util.LoggedTunableNumber;
import com.team6647.util.Constants.ShooterConstants;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class ShooterPivotSubsystem extends SubsystemBase {

  private static ShooterPivotSubsystem instance;

  @AutoLogOutput(key = "Shooter/Pivot/State")
  @Getter private ShooterPivotState mState = ShooterPivotState.HOMED;

  private ShooterPivotIO io;
  private ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

  private LoggedTunableNumber pivotKp = new LoggedTunableNumber("Shooter/Pivot/kp", ShooterConstants.pivotKp);
  private LoggedTunableNumber pivotKi = new LoggedTunableNumber("Shooter/Pivot/ki", ShooterConstants.pivotKi);
  private LoggedTunableNumber pivotKd = new LoggedTunableNumber("Shooter/Pivot/kd", ShooterConstants.pivotKd);
  private LoggedTunableNumber pivotMaxVel = new LoggedTunableNumber("Shooter/Pivot/maxVel",
      ShooterConstants.pivotMaxVel);
  private LoggedTunableNumber pivotMaxAccel = new LoggedTunableNumber("Shooter/Pivot/maxAccel",
      ShooterConstants.pivotMaxAccel);

  private LoggedTunableNumber pivotSetpoint = new LoggedTunableNumber("Shooter/Pivot/Setpoint",
      ShooterConstants.pivotHomedPosition);

  private static ShootingParameters currentParameters;

  @AutoLogOutput(key = "Shooter/Pivot/Emergency Disable")
  private boolean emergencyDisable = false;

  @AutoLogOutput(key = "Shooter/Pivot/Setpoint")
  private double setpoint = ShooterConstants.pivotHomedPosition;

  private Alert pivotEncoderRangeAlert = new Alert("Shooter Pivot Encoder out of range", AlertType.WARNING);
  private Alert pivotEncoderDisconnectedAlert = new Alert("Shooter Pivot Encoder disconnected", AlertType.WARNING);
  private Alert pivotLeftMotorDiconnectedAlert = new Alert("Shooter Pivot Left Motor disconnected", AlertType.WARNING);
  private Alert pivotRightMotorDisconnectedAlert = new Alert("Shooter Pivot Right Motor disconnected",
      AlertType.WARNING);

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(1)), Volts.of(2), Seconds.of(10)),
      new SysIdRoutine.Mechanism(
          (Measure<Voltage> volts) -> {
            runPivotVolts(volts.in(Units.Volts));
          },
          log -> {
            log.motor("pivot")
                .voltage(Volts.of(inputs.shooterPivotRightMotorAppliedVolts))
                .angularPosition(Rotations.of(inputs.shooterPivotRightMotorPosition))
                .angularVelocity(RotationsPerSecond.of(inputs.shooterPivotRightMotorVelocity));
          },
          this));

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
    pivotEncoderRangeAlert.set(inputs.cancoderAbsolutePosition < ShooterConstants.pivotMinPosition
        || inputs.cancoderAbsolutePosition > ShooterConstants.pivotMaxPosition);

    pivotEncoderDisconnectedAlert.set(!inputs.cancoderConnected);
    pivotLeftMotorDiconnectedAlert.set(!inputs.shooterPivotLeftMotorConnected);
    pivotRightMotorDisconnectedAlert.set(!inputs.shooterPivotRightMotorConnected);

    if (inputs.cancoderAbsolutePosition < ShooterConstants.pivotMinPosition
        || inputs.cancoderAbsolutePosition > ShooterConstants.pivotMaxPosition) {
      setShooterPivotState(ShooterPivotState.EMERGENCY_DISABLED);
      io.disablePivot();
    }

    if (!inputs.cancoderConnected) {
      setShooterPivotState(ShooterPivotState.EMERGENCY_DISABLED);
      io.disablePivot();
    }

    if (mState == ShooterPivotState.EMERGENCY_DISABLED) {
      io.setShooterReference(inputs.cancoderAbsolutePosition, false);
      io.disablePivot();
    }

    LoggedTunableNumber.ifChanged(hashCode(), pid -> {
      io.setPIDVel(pid[0], pid[1], pid[2], pid[3], pid[4]);

      changeSetpoint(pid[5], false);
    }, pivotKp, pivotKi, pivotKd, pivotMaxVel, pivotMaxAccel, pivotSetpoint);

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
    EMERGENCY_DISABLED(-1);

    private final double setpoint;
  }

  public void setShooterPivotState(ShooterPivotState state) {
    mState = state;

    switch (state) {
      case HOMED:
        changeSetpoint(state.setpoint, true);
        break;
      case SHOOTING:
        changeSetpoint(currentParameters.pivotAngle(), false);
        break;
      case AMP:
        changeSetpoint(state.setpoint, false);
        break;
      case INDEXING:
        changeSetpoint(state.setpoint, false);
        break;
      case CLIMBING:
        changeSetpoint(state.setpoint, false);
        break;
      case EMERGENCY_DISABLED:
        changeSetpoint(inputs.cancoderAbsolutePosition, false);
        io.disablePivot();
        break;

    }
  }

  /**
   * Public security measure for arbitrarily changing the setpoint
   * 
   * @param newSetpoint The new setpoint
   */
  public void changeSetpoint(double newSetpoint, boolean goingHome) {
    if (newSetpoint > ShooterConstants.pivotMaxPosition || newSetpoint < ShooterConstants.pivotMinPosition) {
      newSetpoint = Functions.clamp(newSetpoint, ShooterConstants.pivotMinPosition,
          ShooterConstants.pivotMaxPosition);
    }

    setpoint = newSetpoint;

    io.setShooterReference(newSetpoint, goingHome);
  }

  public boolean inTolerance() {
    return inputs.inTolerance;
  }

  public static void updateShootingParameters(ShootingParameters newParameters) {
    currentParameters = newParameters;
  }

  public void enablePivot() {
    io.enablePivot();
  }

  /* Characterization */

  public void runPivotVolts(double volts) {
    io.runPivotVolts(volts);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}

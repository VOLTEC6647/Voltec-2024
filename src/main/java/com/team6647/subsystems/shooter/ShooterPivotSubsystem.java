/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivotSubsystem extends SubsystemBase {

  private static ShooterPivotSubsystem instance;

  private static ShooterPivotState mState;

  private ShooterPivotIO io;
  private ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

  /** Creates a new ShooterPivotSubsystem. */
  private ShooterPivotSubsystem(ShooterPivotIO io) {
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

    if (mState == ShooterPivotState.SHOOTING) {
      calculateShooterAngle();
    }
  }

  public enum ShooterPivotState {
    HOMED,
    SHOOTING
  }

  public void setShooterPivotState(ShooterPivotState state) {
    switch (state) {
      case HOMED:
        mState = ShooterPivotState.HOMED;
        setShooterSetpoint(ShooterConstants.shooterHomedPosition);
        break;
      case SHOOTING:
        mState = ShooterPivotState.SHOOTING;
        break;
      default:
        break;
    }
  }

  public void calculateShooterAngle() {

  }

  public void setShooterSetpoint(double setpoint) {

  }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.math.Functions;
import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivotSubsystem extends SubsystemBase {

  private static ShooterPivotSubsystem instance;

  private static ShooterPivotState mState = ShooterPivotState.HOMED;

  private ShooterPivotIO io;
  private ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

  private ProfiledPIDController mPivotController = new ProfiledPIDController(ShooterConstants.pivotKp,
      ShooterConstants.piovtKi, ShooterConstants.piovtKi,
      new TrapezoidProfile.Constraints(ShooterConstants.pivotMaxVelocity, ShooterConstants.pivotMaxAcceleration));

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

    if (mState == ShooterPivotState.SHOOTING) {
      calculateShooterAngle();
    }

    computePID();

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

  public void calculateShooterAngle() {

  }

  private void changeSetpoint(double newSetpoint) {
    if (newSetpoint > ShooterConstants.pivotMaxPosition || newSetpoint < ShooterConstants.pivotMinPosition) {
      newSetpoint = Functions.clamp(newSetpoint, ShooterConstants.pivotMinPosition,
          ShooterConstants.pivotMaxPosition);
    }

    setpoint = newSetpoint;
  }

  public void computePID() {
    double output = mPivotController.calculate(inputs.shooterAbsoluteEncoderPosition, setpoint);

    output = Functions.clamp(output, 0.2, 0.2);

    Logger.recordOutput("Shooter/Pivot/output", output);
    output = output * 12;

    io.setShooterVoltage(output);
  }
}

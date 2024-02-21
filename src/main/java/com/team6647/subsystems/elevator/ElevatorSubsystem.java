/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 01 2024
 */

package com.team6647.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.math.Functions;
import com.team6647.util.Constants.ElevatorConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private static ElevatorSubsystem instance;

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  @AutoLogOutput(key = "Elevator/State")
  private ElevatorState mState = ElevatorState.HOMED;

  private ProfiledPIDController elevatorPIDController = new ProfiledPIDController(ElevatorConstants.elevatorKp,
      ElevatorConstants.elevatorKi, ElevatorConstants.elevatorKd, new TrapezoidProfile.Constraints(1, 1));

  @AutoLogOutput(key = "Elevator/Setpoint")
  private double setpoint = ElevatorConstants.elevatorHomedPosition;

  /** Creates a new ElevatorSubsystem. */
  private ElevatorSubsystem(ElevatorIO io) {
    this.io = io;

    elevatorPIDController.reset(inputs.elevatorAbsoluteEncoderPosition);
  }

  public static ElevatorSubsystem getInstance(ElevatorIO io) {
    if (instance == null) {
      instance = new ElevatorSubsystem(io);
    }
    return instance;
  }

  public enum ElevatorState {
    HOMED,
    TOP,
    AMP,
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    computePID();
  }

  public void changeElevatorState(ElevatorState state) {
    switch (state) {
      case HOMED:
        mState = ElevatorState.HOMED;
        changeSetpoint(ElevatorConstants.elevatorHomedPosition);
        break;
      case TOP:
        mState = ElevatorState.TOP;
        changeSetpoint(ElevatorConstants.elevatorTopPosition);
        break;
      case AMP:
        mState = ElevatorState.AMP;
        changeSetpoint(ElevatorConstants.elevatorAmpPosition);
    }
  }

  private void changeSetpoint(double newSetpoint) {
    if (newSetpoint < ElevatorConstants.elevatorMaxPosition || newSetpoint > ElevatorConstants.elevatorMinPosition) {
      newSetpoint = Functions.clamp(newSetpoint, ElevatorConstants.elevatorMinPosition,
          ElevatorConstants.elevatorMaxPosition);
    }

    setpoint = newSetpoint;
  }

  private void computePID() {
    double volts = elevatorPIDController.calculate(inputs.elevatorPosition, setpoint);

    Logger.recordOutput("Elevator/Output", volts);

    io.setElevatorVoltage(volts);
  }

  @AutoLogOutput(key = "Elevator/InTolerance")
  public boolean inTolerance() {
    return elevatorPIDController.atGoal();
  }
}

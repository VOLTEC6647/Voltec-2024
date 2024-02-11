/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 01 2024
 */

package com.team6647.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team6647.util.Constants.ElevatorConstants;

import edu.wpi.first.math.MathUtil;
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

  /** Creates a new ElevatorSubsystem. */
  private ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
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
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void changeElevatorState(ElevatorState state) {
    switch (state) {
      case HOMED:
        mState = ElevatorState.HOMED;
        setElevatorPosition(ElevatorConstants.elevatorMinPosition);
        break;
      case TOP:
        mState = ElevatorState.TOP;
        setElevatorPosition(ElevatorConstants.elevatorMaxPosition);
        break;
    }
  }

  private void setElevatorPosition(double setpoint) {
    double volts = elevatorPIDController
        .calculate(inputs.elevatorPosition, setpoint);
    volts = MathUtil.clamp(volts, -12.0, 12.0);
    io.setElevatorVoltage(setpoint);
  }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 01 2024
 */
package com.team6647.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team6647.subsystems.shooter.ShooterSubsystem.RollerState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static IntakeSubsystem instance;

  @AutoLogOutput(key = "Intake/State")
  private RollerState mState = RollerState.STOPPED;

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new IntakeSubsystem. */
  private IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  public static IntakeSubsystem getInstance(IntakeIO io) {
    if (instance == null) {
      instance = new IntakeSubsystem(io);
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /**
   * Public method to command intake state
   * 
   * @param rollerState Intake RollerState
   */
  public void changeRollerState(RollerState rollerState) {
    switch (rollerState) {
      case STOPPED:
        mState = RollerState.STOPPED;
        io.setIntakeVelocity(0.0);
        break;
      case EXHAUSTING:
        mState = RollerState.EXHAUSTING;
        io.setIntakeVelocity(0.0);
        break;
      case INTAKING:
        mState = RollerState.INTAKING;
        io.setIntakeVelocity(0.0);
        break;
      case IDLE:
        mState = RollerState.IDLE;
        io.setIntakeVelocity(0.0);
        break;
    }
  }
}
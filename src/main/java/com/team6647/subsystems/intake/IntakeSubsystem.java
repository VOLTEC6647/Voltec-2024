/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 01 2024
 */
package com.team6647.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team6647.util.Constants.IntakeConstants;
import com.team6647.util.Constants.RobotConstants.RollerState;
import com.team6647.util.LoggedTunableNumber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static IntakeSubsystem instance;

  @AutoLogOutput(key = "Intake/Rollers/State")
  private RollerState mState = RollerState.STOPPED;

  private LoggedTunableNumber peakCurrentLimit = new LoggedTunableNumber("Intake/Rollers/PeakCurrentLimit", 4);

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/Roller/Peak")
  private double peakCurrent = 4;

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
    Logger.processInputs("Intake/Rollers", inputs);

    LoggedTunableNumber.ifChanged(hashCode(), peak -> {
      peakCurrent = peak[0];
    }, peakCurrentLimit);
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
        io.setIntakeVelocity(IntakeConstants.intakeStoppedVelocity);
        break;
      case EXHAUSTING:
        mState = RollerState.EXHAUSTING;
        io.setIntakeVelocity(IntakeConstants.intakeExhaustingVelocity);
        break;
      case INTAKING:
        mState = RollerState.INTAKING;
        io.setIntakeVelocity(IntakeConstants.intakeIntakingVelocity);
        break;
      case IDLE:
        mState = RollerState.IDLE;
        io.setIntakeVelocity(IntakeConstants.intakeIdleVelocity);
        break;
    }
  }

  public double getAmps() {
    return inputs.intakeMotorCurrent;
  }

  public boolean objectDetected() {
    return getAmps() > 5;
  }

  public boolean getBeamBrake() {
    return inputs.intakeBeamBrake;
  }
}

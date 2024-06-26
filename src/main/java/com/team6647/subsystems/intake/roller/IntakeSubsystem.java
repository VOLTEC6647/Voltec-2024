/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 01 2024
 */
package com.team6647.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.util.Alert;
import com.andromedalib.util.Alert.AlertType;
import com.team6647.subsystems.leds.LEDSubsystem;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class IntakeSubsystem extends SubsystemBase {

  private static IntakeSubsystem instance;

  @Setter
  @AutoLogOutput(key = "Intake/Rollers/State")
  public IntakeRollerState mState = IntakeRollerState.STOPPED;

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private Alert beamBrakeAlert = new Alert("Intake Beam Brake note detected",
      AlertType.INFO);

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

  @RequiredArgsConstructor
  public enum IntakeRollerState {
    STOPPED(IntakeConstants.intakeStoppedVelocity),
    EXHAUSTING(IntakeConstants.intakeExhaustingVelocity),
    INTAKING(IntakeConstants.intakeIntakingVelocity),
    IDLE(IntakeConstants.intakeIdleVelocity);

    public final double velocity;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Rollers", inputs);

    io.setIntakeVelocity(mState.velocity);

    beamBrakeAlert.set(!getBeamBrake());

    if (!getBeamBrake()) {
      LEDSubsystem.getInstance().intakeHasNote = true;
    } else {
      LEDSubsystem.getInstance().intakeHasNote = false;
    }
  }

  public double getAmps() {
    return inputs.intakeMotorCurrent;
  }

  public boolean getBeamBrake() {
    return inputs.intakeBeamBrake;
  }
}

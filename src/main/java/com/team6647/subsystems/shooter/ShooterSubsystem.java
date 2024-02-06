/**
 * Written by Mauricio Villarreal
 *            Juan Pablo Gutiérrez
 */
package com.team6647.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem instance;

  @AutoLogOutput(key = "Shooter/State")
  private RollerState mState = RollerState.STOPPED;

  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs;

  private ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  public static ShooterSubsystem getInstance(ShooterIO io) {
    if (instance == null) {
      instance = new ShooterSubsystem(io);
    }
    return instance;
  }

  /**
   * Shared between Shooter and Intake
   */
  public enum RollerState {
    STOPPED,
    EXHAUSTING,
    INTAKING,
    IDLE
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Rollers", inputs);
  }

  /**
   * Public method to command shooter state
   * 
   * @param rollerState Shooter RollerState
   */
  public void changeRollerState(RollerState rollerState) {
    switch (rollerState) {
      case STOPPED:
        mState = RollerState.STOPPED;
        setShooterSpeed(ShooterConstants.passiveStopped);
        break;
      case EXHAUSTING:
        mState = RollerState.EXHAUSTING;
        setShooterSpeed(0);
        break;
      case INTAKING:
        mState = RollerState.INTAKING;
        setShooterSpeed(0);
        break;
      case IDLE:
        mState = RollerState.IDLE;
        setShooterSpeed(0);
        break;
    }
  }

  /**
   * Sets the shooter to the desired speed
   * 
   * @param speed Desired speedƒ
   */
  private void setShooterSpeed(double speed) {
    io.setShooterVelocity(speed);
  }

  /**
   * Gets the beam brake status
   * 
   * @return Beam brake state
   */
  @AutoLogOutput(key = "Shooter/BeamBrake")
  public boolean getBeamBrake() {
    return inputs.beamBrake;
  }
}

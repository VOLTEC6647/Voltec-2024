/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 16 02 2024
 */

package com.team6647.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team6647.util.Constants.RobotConstants.RollerState;
import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterRollerSubsystem extends SubsystemBase {

  private static ShooterRollerSubsystem instance;

  private ShooterRollerIO io;
  private ShooterRollerIOInputsAutoLogged inputs = new ShooterRollerIOInputsAutoLogged();

  @AutoLogOutput(key = "Shooter/Roller/State")
  private RollerState mRollerState = RollerState.STOPPED;

  /** Creates a new ShooterRollerSubsystem. */
  private ShooterRollerSubsystem(ShooterRollerIO io) {
    this.io = io;
  }

  public static ShooterRollerSubsystem getInstance(ShooterRollerIO io) {
    if (instance == null) {
      instance = new ShooterRollerSubsystem(io);
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Roller", inputs);
  }

  public void changeRollerState(RollerState rollerState) {
    switch (rollerState) {
      case STOPPED:
        mRollerState = RollerState.STOPPED;
        io.setRollerVelocity(0);
        break;
      case EXHAUSTING:
        mRollerState = RollerState.EXHAUSTING;
        io.setRollerVelocity(ShooterConstants.rollerExhaustingVelocity);
        break;
      case INTAKING:
        mRollerState = RollerState.INTAKING;
        io.setRollerVelocity(ShooterConstants.rollerIntakingVelocity);
        break;
      case IDLE:
        mRollerState = RollerState.IDLE;
        io.setRollerVelocity(ShooterConstants.rollerIdleVelocity);
        break;
    }
  }

  public boolean objectDetected() {
    return inputs.rollerCurrent > 8;
  }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 16 02 2024
 */

package com.team6647.subsystems.shooter.roller;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ShooterRollerSubsystem extends SubsystemBase {

  private static ShooterRollerSubsystem instance;

  private ShooterRollerIO io;
  private ShooterRollerIOInputsAutoLogged inputs = new ShooterRollerIOInputsAutoLogged();

  @Setter
  @AutoLogOutput(key = "Shooter/Roller/State")
  public ShooterFeederState mRollerState = ShooterFeederState.STOPPED;

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

  @RequiredArgsConstructor
  public enum ShooterFeederState {
    STOPPED(ShooterConstants.shooterStoppedSpeed),
    EXHAUSTING(ShooterConstants.rollerExhaustingVelocity),
    INTAKING(ShooterConstants.rollerIntakingVelocity),
    IDLE(ShooterConstants.rollerIdleVelocity);

    public final double velocity;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Roller", inputs);

    io.setRollerVelocity(mRollerState.velocity);
  }

  public double getAmps() {
    return inputs.rollerCurrent;
  }
}

/**
 * Written by Mauricio Villarreal
 *            Juan Pablo Gutiérrez
 */

package com.team6647.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team6647.util.Constants.ShooterConstants;
import com.team6647.util.Constants.RobotConstants.RollerState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem instance;

  @AutoLogOutput(key = "Shooter/Flywheel/State")
  private FlywheelState mFlywheelState = FlywheelState.STOPPED;

  @AutoLogOutput(key = "Shooter/Rollers/State")
  private RollerState mRollerState = RollerState.STOPPED;

  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double mVelocitySetpoint = 0.0;

  private ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  public static ShooterSubsystem getInstance(ShooterIO io) {
    if (instance == null) {
      instance = new ShooterSubsystem(io);
    }
    return instance;
  }

  public enum FlywheelState {
    STOPPED,
    EXHAUSTING,
    SHOOTING,
    IDLE
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel", inputs);
  }

  /**
   * Public method to command shooter state
   * 
   * @param rollerState Shooter RollerState
   */
  public void changeFlywheelState(FlywheelState rollerState) {
    switch (rollerState) {
      case STOPPED:
        mFlywheelState = FlywheelState.STOPPED;
        setShooterSpeed(0);
        break;
      case EXHAUSTING:
        mFlywheelState = FlywheelState.EXHAUSTING;
        setShooterSpeed(0);
        break;
      case SHOOTING:
        mFlywheelState = FlywheelState.SHOOTING;
        setShooterSpeed(0);
        break;
      case IDLE:
        mFlywheelState = FlywheelState.IDLE;
        setShooterSpeed(0);
        break;
    }
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

  /**
   * Sets the shooter to the desired speed
   * 
   * @param speed Desired speedƒ
   */
  private void setShooterSpeed(double speed) {
    io.setShooterVelocity(speed);
  }

  @AutoLogOutput(key = "Shooter/Flywheel/topInTolerance")
  public boolean topInTolerance() {
    return Math.abs(inputs.topMotorVelocity - mVelocitySetpoint) < ShooterConstants.shooterTolerance;
  }

  @AutoLogOutput(key = "Shooter/Flywheel/bottomInTolerance")
  public boolean bottomInTolerance() {
    return Math.abs(inputs.topMotorVelocity - mVelocitySetpoint) < ShooterConstants.shooterTolerance;
  }

}

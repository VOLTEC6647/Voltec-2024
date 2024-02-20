/**
 * Written by Mauricio Villarreal
 *            Juan Pablo Gutiérrez
 */

package com.team6647.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team6647.util.LoggedTunableNumber;
import com.team6647.util.Constants.ShooterConstants;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem instance;

  @AutoLogOutput(key = "Shooter/Flywheel/State")
  private FlywheelState mFlywheelState = FlywheelState.STOPPED;

  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  @AutoLogOutput(key = "Shooter/Flywheel/Setpoint")
  private double mVelocitySetpoint = 0.0;

  private LoggedTunableNumber shooterKp = new LoggedTunableNumber("Shooter/Flywheel/kp", ShooterConstants.shooterKp);
  private LoggedTunableNumber shooterKi = new LoggedTunableNumber("Shooter/Flywheel/ki", ShooterConstants.shooterKi);
  private LoggedTunableNumber shooterKd = new LoggedTunableNumber("Shooter/Flywheel/kd", ShooterConstants.shooterKd);
  private LoggedTunableNumber shooterKf = new LoggedTunableNumber("Shooter/Flywheel/kf", ShooterConstants.shooterKf);
  private LoggedTunableNumber shooterVelocity = new LoggedTunableNumber("Shooter/Flywheel/velocity", 0.0);

  private static ShootingParameters currentParameters = new ShootingParameters(new Rotation2d(), 0, 0);

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

    LoggedTunableNumber.ifChanged(hashCode(), pid -> {
      io.setPIDF(pid[0], pid[1], pid[2], pid[3]);

      setShooterSpeed(pid[4]);
    }, shooterKp, shooterKi, shooterKd, shooterKf, shooterVelocity);
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
        setShooterSpeed(ShooterConstants.shooterStoppedSpeed);
        break;
      case EXHAUSTING:
        mFlywheelState = FlywheelState.EXHAUSTING;
        setShooterSpeed(ShooterConstants.shooterExhaustSpeed);
        break;
      case SHOOTING:
        mFlywheelState = FlywheelState.SHOOTING;
        setShooterSpeed(currentParameters.flywheelRPM());
        break;
      case IDLE:
        mFlywheelState = FlywheelState.IDLE;
        setShooterSpeed(ShooterConstants.shooterIdleSpeed);
        break;
    }
  }

  /**
   * Sets the shooter to the desired speed in RPMs
   * 
   * @param speed Desired speed
   */
  private void setShooterSpeed(double speed) {
    mVelocitySetpoint = speed;
    io.setShooterVelocity(speed);
  }

  public static void updateShootingParameters(ShootingParameters newParameters) {
    currentParameters = newParameters;
  }

  @AutoLogOutput(key = "Shooter/Flywheel/topInTolerance")
  public boolean topInTolerance() {
    return Math.abs(inputs.topMotorVelocity - mVelocitySetpoint) < ShooterConstants.shooterTolerance;
  }

  @AutoLogOutput(key = "Shooter/Flywheel/bottomInTolerance")
  public boolean bottomInTolerance() {
    return Math.abs(inputs.topMotorVelocity - mVelocitySetpoint) < ShooterConstants.shooterTolerance;
  }

  public boolean getBeamBrake() {
    return inputs.beamBrake;
  }

}

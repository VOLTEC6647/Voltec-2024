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

  private LoggedTunableNumber bottomShooterKp = new LoggedTunableNumber("Shooter/Flywheel/bottomKp",
      ShooterConstants.bottomShooterKp);
  private LoggedTunableNumber bottomShooterKi = new LoggedTunableNumber("Shooter/Flywheel/bottomKi",
      ShooterConstants.bottomShooterKi);
  private LoggedTunableNumber bottomShooterKd = new LoggedTunableNumber("Shooter/Flywheel/bottomKd",
      ShooterConstants.bottomShooterKd);
  private LoggedTunableNumber bottomShooterKs = new LoggedTunableNumber("Shooter/Flywheel/bottomKs",
      ShooterConstants.bottomShooterKs);
  private LoggedTunableNumber bottomShooterKv = new LoggedTunableNumber("Shooter/Flywheel/bottomKv",
      ShooterConstants.bottomShooterKv);
  private LoggedTunableNumber bottomShooterKa = new LoggedTunableNumber("Shooter/Flywheel/bottomKd",
      ShooterConstants.bottomShooterKa);

  private LoggedTunableNumber topShooterKp = new LoggedTunableNumber("Shooter/Flywheel/topKp",
      ShooterConstants.topShooterKp);
  private LoggedTunableNumber topShooterKi = new LoggedTunableNumber("Shooter/Flywheel/topKi",
      ShooterConstants.topShooterKi);
  private LoggedTunableNumber topShooterKd = new LoggedTunableNumber("Shooter/Flywheel/topKd",
      ShooterConstants.topShooterKd);
  private LoggedTunableNumber topShooterKs = new LoggedTunableNumber("Shooter/Flywheel/topKs",
      ShooterConstants.topShooterKs);
  private LoggedTunableNumber topShooterKv = new LoggedTunableNumber("Shooter/Flywheel/topKv",
      ShooterConstants.topShooterKv);
  private LoggedTunableNumber topShooterKa = new LoggedTunableNumber("Shooter/Flywheel/topKd",
      ShooterConstants.topShooterKa);

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
      io.setBottomPIDF(pid[0], pid[1], pid[2], pid[3], pid[4], pid[5]);

      io.setTopPIDF(pid[6], pid[7], pid[8], pid[9], pid[10], pid[11]);

      setShooterSpeed(pid[12]);
    }, bottomShooterKp, bottomShooterKi, bottomShooterKd, bottomShooterKs, bottomShooterKv, bottomShooterKa,
        topShooterKp, topShooterKi, topShooterKd, topShooterKs, topShooterKv, topShooterKa,
        shooterVelocity);
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
        setShooterSpeed(1000);
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

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.math.Functions;
import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterPivotSubsystem extends SubsystemBase {

  private static ShooterPivotSubsystem instance;

  @AutoLogOutput(key = "Shooter/Pivot/State")
  private static ShooterPivotState mState = ShooterPivotState.HOMED;

  private ShooterPivotIO io;
  private ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

  private ProfiledPIDController mPivotController = new ProfiledPIDController(ShooterConstants.pivotKp,
      ShooterConstants.piovtKi, ShooterConstants.pivotKd,
      new TrapezoidProfile.Constraints(ShooterConstants.pivotMaxVelocity, ShooterConstants.pivotMaxAcceleration));

  @AutoLogOutput(key = "Shooter/Pivot/Setpoint")
  private double setpoint = ShooterConstants.pivotHomedPosition;

  private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = MutableMeasure.mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = MutableMeasure.mutable(RotationsPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.5).per(Seconds.of(1)), Volts.of(3), Seconds.of(60), null),
      new SysIdRoutine.Mechanism(
          (Measure<Voltage> volts) -> {
            runPivotCharacterization(volts);
          },
          log -> {
            log.motor("pivot")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        inputs.shooterPivotAppliedVolts, Volts))
                .angularPosition(m_angle.mut_replace(inputs.shooterAbsoluteEncoderPosition, Rotations))
                .angularVelocity(m_velocity.mut_replace(inputs.shooterAbsoluteEncoderVelocity, RotationsPerSecond));
          },
          this));

  /** Creates a new ShooterPivotSubsystem. */
  private ShooterPivotSubsystem(
      ShooterPivotIO io) {
    this.io = io;

    mPivotController.reset(setpoint);
  }

  public static ShooterPivotSubsystem getInstance(ShooterPivotIO io) {
    if (instance == null) {
      instance = new ShooterPivotSubsystem(io);
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Pivot", inputs);

    if (mState == ShooterPivotState.SHOOTING) {
      calculateShooterAngle();
    }

    computePID();

  }

  public enum ShooterPivotState {
    HOMED,
    SHOOTING,
    INDEXING,
  }

  public void setShooterPivotState(ShooterPivotState state) {
    switch (state) {
      case HOMED:
        mState = ShooterPivotState.HOMED;
        changeSetpoint(ShooterConstants.pivotHomedPosition);
        break;
      case SHOOTING:
        mState = ShooterPivotState.SHOOTING;
        break;
      case INDEXING:
        mState = ShooterPivotState.INDEXING;
        changeSetpoint(ShooterConstants.pivotIndexingPosition);
      default:
        break;
    }
  }

  public void calculateShooterAngle() {

  }

  private void changeSetpoint(double newSetpoint) {
    if (newSetpoint > ShooterConstants.pivotMaxPosition || newSetpoint < ShooterConstants.pivotMinPosition) {
      newSetpoint = Functions.clamp(newSetpoint, ShooterConstants.pivotMinPosition,
          ShooterConstants.pivotMaxPosition);
    }

    setpoint = newSetpoint;
  }

  public void computePID() {
    double output = mPivotController.calculate(inputs.shooterAbsoluteEncoderPosition, setpoint);
    Logger.recordOutput("Shooter/Pivot/output", output);

    Logger.recordOutput("Shooter/Pivot/ClampedOutput", output);
    output = output * 12;

    io.setShooterVoltage(output);
  }

  public boolean inTolerance() {
    return mPivotController.atGoal();
  }

  public void runPivotCharacterization(Measure<Voltage> volts) {
    io.setShooterVoltage(volts.baseUnitMagnitude());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}

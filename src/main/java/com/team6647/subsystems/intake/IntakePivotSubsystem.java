/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 26 01 2024
 */
package com.team6647.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.math.Functions;
import com.team6647.util.Constants.IntakeConstants;

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

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class IntakePivotSubsystem extends SubsystemBase {

  private static IntakePivotSubsystem instance;

  @AutoLogOutput(key = "Intake/Pivot/State")
  private IntakePivotState mState = IntakePivotState.HOMED;

  private IntakePivotIO io;
  private IntakePivoIOInputsAutoLogged inputs = new IntakePivoIOInputsAutoLogged();

  private ProfiledPIDController mIntakePivotController = new ProfiledPIDController(IntakeConstants.pivotKp,
      IntakeConstants.pivotKi, IntakeConstants.pivotKd,
      new TrapezoidProfile.Constraints(IntakeConstants.intakePIDMaxVelocity,
          IntakeConstants.intakePIDMaxAcceleration));

  @AutoLogOutput(key = "Intake/Pivot/Setpoint")
  private double setpoint = IntakeConstants.intakeHomedPosition;

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.5).per(Seconds.of(1)), Volts.of(3), Seconds.of(60), null),
      new SysIdRoutine.Mechanism(
          (Measure<Voltage> volts) -> {
            runIntakeCharacterization(volts);
          },
          log -> {
            log.motor("intake-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        inputs.intakePivotLeftMotorAppliedVoltage, Volts))
                .angularPosition(m_angle.mut_replace(inputs.intakePivotLeftMotorPosition, Rotations))
                .angularVelocity(m_velocity.mut_replace(inputs.intakePivotLeftMotorVelocity, RotationsPerSecond));
            log.motor("intake-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        inputs.intakePivotRightMotorAppliedVoltage, Volts))
                .angularPosition(m_angle.mut_replace(inputs.intakePivotRightMotorPosition, Rotations))
                .angularVelocity(m_velocity.mut_replace(inputs.intakePivotRightMotorVelocity, RotationsPerSecond));
          },
          this));

  /** Creates a new IntakePivotSubsystem. */
  private IntakePivotSubsystem(IntakePivotIO io) {
    this.io = io;

    mIntakePivotController.setTolerance(IntakeConstants.intakePivotPositionTolerance);

    resetPID();
  }

  public static IntakePivotSubsystem getInstance(IntakePivotIO io) {
    if (instance == null) {
      instance = new IntakePivotSubsystem(io);
    }
    return instance;
  }

  public enum IntakePivotState {
    HOMED,
    EXTENDED
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Pivot", inputs);

    computePID(mState == IntakePivotState.HOMED);
  }

  public void changeIntakePivotState(IntakePivotState intakePivotState) {
    switch (intakePivotState) {
      case HOMED:
        mState = IntakePivotState.HOMED;
        changeSetpoint(IntakeConstants.intakeHomedPosition);
        break;
      case EXTENDED:
        mState = IntakePivotState.EXTENDED;
        changeSetpoint(IntakeConstants.intakeExtendedPosition);
        break;
      default:
        break;
    }
  }

  private void changeSetpoint(double newSetpoint) {
    if (newSetpoint < IntakeConstants.maxIntakePivotPosition || newSetpoint > IntakeConstants.minIntakePivotPosition) {
      newSetpoint = Functions.clamp(newSetpoint, IntakeConstants.minIntakePivotPosition,
          IntakeConstants.maxIntakePivotPosition);
    }

    setpoint = newSetpoint;
  }

  private void computePID(boolean equalOutput) {
    double output = mIntakePivotController.calculate(inputs.intakePivotAbsoluteEncoderPosition, setpoint);

    double feedforwardValue = equalOutput ? output : output * 0.8;

    output = output * 12;
    feedforwardValue = feedforwardValue * 12;

    Logger.recordOutput("Intake/Pivot/output", output);
    Logger.recordOutput("Intake/Pivot/feedforward", feedforwardValue);

    io.setIntakeVoltage(feedforwardValue, output);
  }

  @AutoLogOutput(key = "Intake/Pivot/InTolerance")
  public boolean inTolerance() {
    return mIntakePivotController.atSetpoint();
  }

  public void resetPID() {
    mIntakePivotController.reset(inputs.intakePivotAbsoluteEncoderPosition);
  }

  /* Characterization */

  private void runIntakeCharacterization(Measure<Voltage> volts) {
    io.setIntakeVoltage(volts.baseUnitMagnitude(), volts.baseUnitMagnitude());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}

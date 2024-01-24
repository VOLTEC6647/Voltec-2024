/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 01 2024
 */

package com.team6647.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team6647.util.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private static ElevatorSubsystem instance;

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  @AutoLogOutput(key = "Elevator/State")
  private ElevatorState mState = ElevatorState.HOMED;

  // TODO configure Mechanism2D approriately
  @AutoLogOutput(key = "Elevator/Mechanism2D")
  private Mechanism2d elevatorMechanism2d;
  private MechanismRoot2d elevatorRoot;
  private MechanismLigament2d elevatorLigament;

  /** Creates a new ElevatorSubsystem. */
  private ElevatorSubsystem(ElevatorIO io) {
    this.io = io;

    elevatorMechanism2d = new Mechanism2d(1, ElevatorConstants.elevatorMaxPosition);
    elevatorRoot = elevatorMechanism2d.getRoot("Elevator", 0.5, 0);
    elevatorLigament = elevatorRoot.append(new MechanismLigament2d("Elevator", 0, 90, 3, new Color8Bit(Color.kRed)));
    elevatorLigament.setLength(ElevatorConstants.elevatorMinPosition);
  }

  public static ElevatorSubsystem getInstance(ElevatorIO io) {
    if (instance == null) {
      instance = new ElevatorSubsystem(io);
    }
    return instance;
  }

  public enum ElevatorState {
    HOMED,
    TOP,
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    elevatorLigament.setLength(inputs.elevatorPosition);
  }

  public void changeElevatorState(ElevatorState state) {
    switch (state) {
      case HOMED:
        mState = ElevatorState.HOMED;
        io.setElevatorPosition(ElevatorConstants.elevatorMinPosition);
        break;
      case TOP:
        mState = ElevatorState.TOP;
        io.setElevatorPosition(ElevatorConstants.elevatorMaxPosition);
        break;
    }
  }
}

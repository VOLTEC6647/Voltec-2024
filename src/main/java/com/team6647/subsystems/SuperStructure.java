/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 11 02 2024
 * 
 * Manages the subsystem state machines. Acts as a robot-wide state machine that controls each mechanism's independent state machine
 */
package com.team6647.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.andromedalib.util.AllianceFlipUtil;
import com.team6647.RobotContainer;
import com.team6647.commands.ElevatorTarget;
import com.team6647.commands.FlywheelTarget;
import com.team6647.commands.InitIntake;
import com.team6647.commands.IntakeHome;
import com.team6647.commands.IntakeRollerTarget;
import com.team6647.commands.ShooterPivotTarget;
import com.team6647.commands.ShooterRollerTarget;
import com.team6647.commands.ShootingStationary;
import com.team6647.commands.VisionIntakeAlign;
import com.team6647.subsystems.elevator.ElevatorSubsystem;
import com.team6647.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;
import com.team6647.subsystems.intake.IntakeCommands;
import com.team6647.subsystems.intake.IntakePivotSubsystem;
import com.team6647.subsystems.intake.IntakeSubsystem;
import com.team6647.subsystems.neural.NeuralVisionSubsystem;
import com.team6647.subsystems.shooter.ShooterCommands;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.ShooterRollerSubsystem;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem.ShooterPivotState;
import com.team6647.subsystems.vision.VisionSubsystem;
import com.team6647.util.Constants.FieldConstants;
import com.team6647.util.Constants.ShooterConstants;
import com.team6647.util.Constants.RobotConstants.RollerState;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class SuperStructure {

    private static SuperStructure instance;

    private static AndromedaSwerve andromedaSwerve = RobotContainer.andromedaSwerve;
    private static ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
    private static ShooterRollerSubsystem rollerSubsystem = RobotContainer.shooterRollerSubsystem;
    private static ShooterPivotSubsystem shooterPivotSubsystem = RobotContainer.shooterPivotSubsystem;
    private static IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
    private static IntakePivotSubsystem intakePivotSubsystem = RobotContainer.intakePivotSubsystem;
    private static VisionSubsystem visionSubsystem = RobotContainer.visionSubsytem;
    private static NeuralVisionSubsystem neuralVisionSubsystem = RobotContainer.neuralVisionSubsystem;
    private static ElevatorSubsystem elevatorSubsystem = RobotContainer.elevatorSubsystem;

    @AutoLogOutput(key = "SuperStructure/State")
    private static SuperStructureState mRobotState = SuperStructureState.IDLE;

    public static SuperStructure getInstance() {
        if (instance == null) {
            instance = new SuperStructure();
        }
        return instance;
    }

    public enum SuperStructureState {
        IDLE, INTAKING, SHOOTING_SPEAKER, SCORING_AMP, SHOOTING_TRAP, SHOOTING_MOVING, CLIMBING, INTAKE_ALIGN
    }

    public static Command update(SuperStructureState newState) {
        switch (newState) {
            case IDLE:
                mRobotState = SuperStructureState.IDLE;
                return idleCommand();
            case INTAKING:
                mRobotState = SuperStructureState.INTAKING;
                return intakingCommand();
            case SHOOTING_SPEAKER:
                mRobotState = SuperStructureState.SHOOTING_SPEAKER;
                return shootingStationary();
            case SCORING_AMP:
                mRobotState = SuperStructureState.SCORING_AMP;
                return scoreAmp();
            case SHOOTING_TRAP:
                return Commands.waitSeconds(0);
            case SHOOTING_MOVING:
                mRobotState = SuperStructureState.SHOOTING_MOVING;
                return shootingWhileMoving();
            case CLIMBING:
                mRobotState = SuperStructureState.CLIMBING;
                return elevatorClimb();
            case INTAKE_ALIGN:
                mRobotState = SuperStructureState.INTAKE_ALIGN;
                return new VisionIntakeAlign(neuralVisionSubsystem,
                        andromedaSwerve);
        }

        return Commands.waitSeconds(0);
    }

    private static Command intakingCommand() {

        return Commands.deadline(
                ShooterCommands.getShooterIntakingCommand(),
                Commands.sequence(
                        IntakeCommands.getIntakeCommand(),
                        Commands.waitSeconds(0.5),
                        new RunCommand(() -> intakeSubsystem.changeRollerState(RollerState.EXHAUSTING), intakeSubsystem)
                                .withTimeout(0.2)));
    }

    private static Command idleCommand() {

        return Commands.sequence(
                new InitIntake(intakePivotSubsystem),
                Commands.parallel(
                        Commands.waitSeconds(0.4).andThen(new IntakeHome(intakePivotSubsystem)),
                        new IntakeRollerTarget(intakeSubsystem, RollerState.STOPPED),
                        new ElevatorTarget(elevatorSubsystem, ElevatorState.HOMED),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.HOMED),
                        new ShooterRollerTarget(rollerSubsystem, RollerState.STOPPED),
                        new FlywheelTarget(shooterSubsystem, FlywheelState.STOPPED)));
    }

    private static Command shootingWhileMoving() {

        return Commands.waitSeconds(0);
    }

    private static Command shootingStationary() {

        return new ShootingStationary(andromedaSwerve, shooterSubsystem, shooterPivotSubsystem, rollerSubsystem,
                visionSubsystem, true);
    }

    /* Pathfinding */

    public static Command goToAmp() {
        return andromedaSwerve.getPathFindPath(AllianceFlipUtil.apply(FieldConstants.amp));
    }

    private static Command scoreAmp() {
        ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), ShooterConstants.pivotAmpPosition,
                ShooterConstants.flywheelAmpRPM);

        updateShootingParameters(ampParams);

        return Commands.sequence(
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.AMP).withTimeout(3),
                new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                new ShooterRollerTarget(rollerSubsystem, RollerState.INTAKING));
    }

    private static Command elevatorClimb() {
        return Commands.parallel(
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.CLIMBING),
                new ElevatorTarget(elevatorSubsystem, ElevatorState.TOP));
    }

    /* Util */

    public static void updateShootingParameters(ShootingParameters newParameters) {
        ShooterSubsystem.updateShootingParameters(newParameters);
        ShooterPivotSubsystem.updateShootingParameters(newParameters);
    }

}

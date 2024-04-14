/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 11 02 2024
 * 
 * Manages the subsystem state machines. Acts as a robot-wide state machine that controls each mechanism's independent state machine
 */
package com.team6647.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.team6647.RobotContainer;
import com.team6647.RobotState;
import com.team6647.commands.FlywheelTarget;
import com.team6647.commands.InitIntake;
import com.team6647.commands.IntakeHome;
import com.team6647.commands.IntakeRollerTarget;
import com.team6647.commands.ShooterPivotTarget;
import com.team6647.commands.ShooterRollerTarget;
import com.team6647.commands.VisionIntakeAlign;
import com.team6647.commands.VisionSpeakerAlign;
import com.team6647.subsystems.drive.Drive;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;
import com.team6647.subsystems.intake.IntakeCommands;
import com.team6647.subsystems.intake.pivot.IntakePivotSubsystem;
import com.team6647.subsystems.intake.roller.IntakeSubsystem;
import com.team6647.subsystems.intake.roller.IntakeSubsystem.IntakeRollerState;
import com.team6647.subsystems.neural.NeuralVisionSubsystem;
import com.team6647.subsystems.shooter.ShooterCommands;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem.ShooterPivotState;
import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem;
import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem.ShooterFeederState;
import com.team6647.subsystems.vision.VisionSubsystem;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.FieldConstants.Speaker;
import com.team6647.util.Constants.ShooterConstants;
import com.team6647.util.AllianceFlipUtil;
import com.team6647.util.ShootingCalculatorUtil;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperStructure {

    private static SuperStructure instance;

    private static Drive andromedaSwerve = RobotContainer.andromedaSwerve;
    private static ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
    private static ShooterRollerSubsystem rollerSubsystem = RobotContainer.shooterRollerSubsystem;
    private static ShooterPivotSubsystem shooterPivotSubsystem = RobotContainer.shooterPivotSubsystem;
    private static IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
    private static IntakePivotSubsystem intakePivotSubsystem = RobotContainer.intakePivotSubsystem;
    private static VisionSubsystem visionSubsystem = RobotContainer.visionSubsytem;
    private static NeuralVisionSubsystem neuralVisionSubsystem = RobotContainer.neuralVisionSubsystem;

    @AutoLogOutput(key = "SuperStructure/State")
    private static SuperStructureState mRobotState = SuperStructureState.IDLE;

    public static SuperStructure getInstance() {
        if (instance == null) {
            instance = new SuperStructure();
        }
        return instance;
    }

    public enum SuperStructureState {
        IDLE,
        AUTO_IDLE,
        INTAKING_COMPLETE,
        INTAKING,
        INDEXING,
        AUTO_INTAKING,
        AUTO_AMP,
        SHOOTING_SPEAKER,
        SHOOTING_SUBWOOFER,
        AUTO_SHOOTING_SUBWOOFER,
        SHUTTLE,
        SCORING_AMP,
        PREPARING_AMP,
        SHOOTING_TRAP,
        SHOOTING_MOVING,
        STOPPING_CLIMB,
        INTAKE_ALIGN
    }

    public static Command update(SuperStructureState newState) {
        switch (newState) {
            case IDLE:
                return idleCommand();
            case AUTO_IDLE:
                return autoIdleCommand();
            case INTAKING_COMPLETE:
                return fullIntakingCommand();
            case INTAKING:
                return intakingCommand();
            case INDEXING:
                return indexingCommand();
            case AUTO_INTAKING:
                return autoIntakingCommand();
            case AUTO_AMP:
                return autoScoreAmp();
            case SHOOTING_SPEAKER:
                return shootingStationary();
            case SHOOTING_SUBWOOFER:
                return shootingSubwoofer();
            case AUTO_SHOOTING_SUBWOOFER:
                return autoShootingSubwoofer();
            case SCORING_AMP:
                return scoreAmp();
            case PREPARING_AMP:
                return prepareScoreAmp();
            case SHOOTING_TRAP:
                return Commands.waitSeconds(0);
            case SHOOTING_MOVING:
                return shootingWhileMoving();
            case STOPPING_CLIMB:
                return homeElevator();
            case INTAKE_ALIGN:
                return new VisionIntakeAlign(neuralVisionSubsystem,
                        andromedaSwerve);
            case SHUTTLE:
                return sendNotes();
            default:
                break;
        }

        return Commands.waitSeconds(0);
    }

    private static Command autoShootingSubwoofer() {
        return Commands.deadline(
                Commands.waitUntil(() -> shooterSubsystem.getBeamBrake()),
                Commands.sequence(
                        setGoalCommand(SuperStructureState.SHOOTING_SUBWOOFER),
                        new InstantCommand(() -> {
                            ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), -45, 3000);

                            updateShootingParameters(ampParams);
                        }),
                        Commands.parallel(
                                new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                        new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING)));
    }

    private static Command setGoalCommand(SuperStructureState state) {
        return new InstantCommand(() -> mRobotState = state);
    }

    private static Command fullIntakingCommand() {
        return Commands.deadline(
                ShooterCommands.getShooterIntakingCommand(),
                setGoalCommand(SuperStructureState.INTAKING_COMPLETE),
                Commands.sequence(
                        IntakeCommands.getFullIntakeCommand(),
                        Commands.waitSeconds(0.5)))
                .andThen(SuperStructure.update(SuperStructureState.IDLE));
    }

    private static Command intakingCommand() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.INTAKING),
                IntakeCommands.getIntakeCommand(),
                Commands.waitSeconds(0.5))
                .andThen(SuperStructure.update(SuperStructureState.IDLE));
    }

    private static Command indexingCommand() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.INDEXING),
                new IntakeHome(intakePivotSubsystem),
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.INDEXING),
                Commands.deadline(
                        ShooterCommands.getShooterIntakingCommand(),
                        new IntakeRollerTarget(
                                intakeSubsystem,
                                IntakeRollerState.INTAKING)),
                Commands.waitSeconds(0.5))
                .andThen(SuperStructure.update(SuperStructureState.IDLE));
    }

    private static Command autoIntakingCommand() {
        return Commands.deadline(
                ShooterCommands.getShooterIntakingCommand(),
                setGoalCommand(SuperStructureState.AUTO_INTAKING),
                Commands.sequence(
                        IntakeCommands.getFullIntakeCommand(),
                        Commands.waitSeconds(0.5)))
                .andThen(SuperStructure.update(SuperStructureState.AUTO_IDLE));
    }

    private static Command idleCommand() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.IDLE),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.STOPPED),
                new InitIntake(intakePivotSubsystem),
                Commands.parallel(
                        Commands.waitSeconds(0.4).andThen(new IntakeHome(intakePivotSubsystem)),
                        new IntakeRollerTarget(intakeSubsystem, IntakeRollerState.STOPPED),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.HOMED),
                        new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.STOPPED),
                        new FlywheelTarget(shooterSubsystem, FlywheelState.STOPPED)));
    }

    private static Command autoIdleCommand() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.AUTO_IDLE),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.STOPPED),
                new InitIntake(intakePivotSubsystem),
                Commands.parallel(
                        Commands.waitSeconds(0.4).andThen(new IntakeHome(intakePivotSubsystem)),
                        new IntakeRollerTarget(intakeSubsystem, IntakeRollerState.STOPPED),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.HOMED).withTimeout(0.1),
                        new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.STOPPED),
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING).withTimeout(0.5)));
    }

    private static Command homeElevator() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.STOPPING_CLIMB),
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.HOMED));
    }

    private static Command shootingWhileMoving() {

        return Commands.waitSeconds(0);
    }

    private static Command shootingStationary() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.SHOOTING_SPEAKER),
                new InstantCommand(() -> {
                    ShootingParameters ampParams = ShootingCalculatorUtil.getShootingParameters(
                            RobotState.getPose(),
                            AllianceFlipUtil.apply(Speaker.centerSpeakerOpening.toTranslation2d()));

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new VisionSpeakerAlign(andromedaSwerve, visionSubsystem),
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    private static Command shootingSubwoofer() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.SHOOTING_SUBWOOFER),
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), -45, 3000);

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    private static Command sendNotes() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.SHUTTLE),
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), -45, 2500);

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    /* Pathfinding */

    public static Command goToAmp() {
        return Commands.sequence(
                andromedaSwerve.getPathFindThenFollowPath("TeleopAmp", DriveConstants.pathFindingConstraints),
                prepareScoreAmp(),
                Commands.waitSeconds(1),
                setGoalCommand(SuperStructureState.SCORING_AMP),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    private static Command autoScoreAmp() {
        return Commands.deadline(
                Commands.waitUntil(() -> shooterSubsystem.getBeamBrake()),
                Commands.sequence(
                        prepareScoreAmp(),
                        Commands.waitSeconds(1),
                        setGoalCommand(SuperStructureState.AUTO_AMP),
                        new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING)));
    }

    private static Command prepareScoreAmp() {
        return Commands.sequence(
                setGoalCommand(SuperStructureState.PREPARING_AMP),
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(),
                            ShooterConstants.pivotAmpPosition,
                            ShooterConstants.flywheelAmpRPM);

                    updateShootingParameters(ampParams);
                }),
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.AMP).withTimeout(1),
                new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING).withTimeout(1));
    }

    private static Command scoreAmp() {
        return Commands.sequence(
                prepareScoreAmp(),
                Commands.waitSeconds(1),
                setGoalCommand(SuperStructureState.SCORING_AMP),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    /* Util */

    public static void updateShootingParameters(ShootingParameters newParameters) {
        ShooterSubsystem.updateShootingParameters(newParameters);
        ShooterPivotSubsystem.updateShootingParameters(newParameters);
    }

    public static Command autoBottomCommand() {
        return Commands.sequence(
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), -25, 5000);

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    public static Command autoMiddleCommand() {
        return Commands.sequence(
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), -45, 5000);

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }

    public static Command autoTopCommand() {
        return Commands.sequence(
                new InstantCommand(() -> {
                    ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), -25, 5000);

                    updateShootingParameters(ampParams);
                }),
                Commands.parallel(
                        new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING),
                        new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.SHOOTING)),
                new ShooterRollerTarget(rollerSubsystem, ShooterFeederState.INTAKING));
    }
}

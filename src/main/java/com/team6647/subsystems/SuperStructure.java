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
import com.andromedalib.math.GeomUtil;
import com.andromedalib.util.AllianceFlipUtil;
import com.team6647.RobotContainer;
import com.team6647.commands.FlywheelTarget;
import com.team6647.commands.IntakePivotTarget;
import com.team6647.commands.IntakeRollerTarget;
import com.team6647.commands.ShooterPivotTarget;
import com.team6647.commands.ShooterRollerTarget;
import com.team6647.commands.ShootingStationary;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;
import com.team6647.subsystems.intake.IntakeCommands;
import com.team6647.subsystems.intake.IntakePivotSubsystem;
import com.team6647.subsystems.intake.IntakeSubsystem;
import com.team6647.subsystems.intake.IntakePivotSubsystem.IntakePivotState;
import com.team6647.subsystems.shooter.ShooterCommands;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.ShooterRollerSubsystem;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem.ShooterPivotState;
import com.team6647.util.Constants.FieldConstants;
import com.team6647.util.Constants.ShooterConstants;
import com.team6647.util.Constants.RobotConstants.RollerState;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SuperStructure {

    private static SuperStructure instance;

    private AndromedaSwerve andromedaSwerve = RobotContainer.andromedaSwerve;
    private ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
    private ShooterRollerSubsystem rollerSubsystem = RobotContainer.shooterRollerSubsystem;
    private ShooterPivotSubsystem shooterPivotSubsystem = RobotContainer.shooterPivotSubsystem;
    private IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
    private IntakePivotSubsystem intakePivotSubsystem = RobotContainer.intakePivotSubsystem;

    @AutoLogOutput(key = "SuperStructure/State")
    private SuperStructureState mRobotState = SuperStructureState.IDLE;

    public static SuperStructure getInstance() {
        if (instance == null) {
            instance = new SuperStructure();
        }
        return instance;
    }

    public enum SuperStructureState {
        IDLE, INTAKING, SHOOTING_SPEAKER, SCORING_AMP, SHOOTING_TRAP, SHOOTING_MOVING, CLIMBING
    }

    public Command update(SuperStructureState newState) {
        switch (newState) {
            case IDLE:
                return idleCommand();
            case INTAKING:
                return intakingCommand();
            case SHOOTING_SPEAKER:
                return shootingStationary();
            case SCORING_AMP:
                return scoreAmp();
            case SHOOTING_TRAP:
                break;
            case SHOOTING_MOVING:
                return shootingWhileMoving();
            case CLIMBING:
                break;
        }

        return Commands.waitSeconds(0);
    }

    private Command intakingCommand() {
        mRobotState = SuperStructureState.INTAKING;

        return Commands.deadline(
                ShooterCommands.getShooterIntakingCommand(),
                IntakeCommands.getIntakeCommand());
    }

    private Command idleCommand() {
        mRobotState = SuperStructureState.INTAKING;

        return Commands.parallel(
                new IntakePivotTarget(intakePivotSubsystem, IntakePivotState.HOMED),
                new IntakeRollerTarget(intakeSubsystem, RollerState.STOPPED),
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.HOMED),
                new ShooterRollerTarget(rollerSubsystem, RollerState.STOPPED),
                new FlywheelTarget(shooterSubsystem, FlywheelState.STOPPED));
    }

    public Command shootingWhileMoving() {
        return Commands.waitSeconds(0);
    }

    public Command shootingStationary() {
        return new ShootingStationary(andromedaSwerve, shooterSubsystem, shooterPivotSubsystem, rollerSubsystem,
                instance);
    }

    /* Pathfinding */

    public Command goToAmp() {
        return andromedaSwerve.getPathFindPath(GeomUtil.toPose2d(AllianceFlipUtil.apply(FieldConstants.ampCenter)));
    }

    public Command goToSpeaker() {
        return andromedaSwerve.getPathFindPath(GeomUtil
                .toPose2d(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d())));
    }

    public Command scoreAmp() {

        ShootingParameters ampParams = new ShootingParameters(new Rotation2d(), ShooterConstants.pivotAmpPosition,
                ShooterConstants.flywheelAmpRPM);

        updateShootingParameters(ampParams);

        return Commands.sequence(
                new ShooterPivotTarget(shooterPivotSubsystem, ShooterPivotState.AMP),
                new ShooterRollerTarget(rollerSubsystem, RollerState.INTAKING),
                new FlywheelTarget(shooterSubsystem, FlywheelState.SHOOTING));
    }

    /* Util */

    public void updateShootingParameters(ShootingParameters newParameters) {
        ShooterSubsystem.updateShootingParameters(newParameters);
        ShooterPivotSubsystem.updateShootingParameters(newParameters);
    }

}

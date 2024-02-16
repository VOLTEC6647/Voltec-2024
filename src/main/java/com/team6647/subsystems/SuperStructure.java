/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 11 02 2024
 * 
 * Manages the subsystem state machines. Acts as a robot-wide state machine that controls each mechanism's independent state machine
 */
package com.team6647.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.andromedalib.andromedaSwerve.commands.SwerveDriveCommand;
import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.andromedalib.math.GeomUtil;
import com.team6647.RobotContainer;
import com.team6647.subsystems.intake.IntakeCommands;
import com.team6647.subsystems.intake.IntakePivotSubsystem.IntakePivotState;
import com.team6647.subsystems.shooter.ShooterCommands;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem.ShooterPivotState;
import com.team6647.util.AllianceFlipUtil;
import com.team6647.util.ShootingCalculatorUtil;
import com.team6647.util.Constants.FieldConstants;
import com.team6647.util.Constants.RobotConstants.RollerState;
import com.team6647.util.ShootingCalculatorUtil.ShootingParameters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SuperStructure {

    private static SuperStructure instance;

    private AndromedaSwerve andromedaSwerve = RobotContainer.andromedaSwerve;

    @AutoLogOutput(key = "SuperStructure/State")
    private SuperStructureState mRobotState = SuperStructureState.IDLE;

    private ShootingParameters currentParameters;

    public static SuperStructure getInstance() {
        if (instance == null) {
            instance = new SuperStructure();
        }
        return instance;
    }

    public enum SuperStructureState {
        IDLE, INTAKING, SHOOTING_SPEAKER, SHOOTING_AMP, SHOOTING_TRAP, SHOOTING_MOVING, CLIMBING
    }

    public Command update(SuperStructureState newState) {
        switch (newState) {
            case IDLE:
                return idleCommand();
            case INTAKING:
                return intakingCommand();
            case SHOOTING_SPEAKER:
                break;
            case SHOOTING_AMP:
                break;
            case SHOOTING_TRAP:
                break;
            case SHOOTING_MOVING:
                break;
            case CLIMBING:
                break;
        }

        return Commands.waitSeconds(0);
    }

    private Command intakingCommand() {
        mRobotState = SuperStructureState.INTAKING;

        return Commands.parallel(
                IntakeCommands.getTargetPivotStateCommand(IntakePivotState.EXTENDED),
                IntakeCommands.getTargetStateIntakeCommand(RollerState.INTAKING),
                ShooterCommands.getTargetRollersCommand(RollerState.INTAKING),
                ShooterCommands.getTargetShooterPivotCommand(ShooterPivotState.INDEXING));
    }

    private Command idleCommand() {
        mRobotState = SuperStructureState.INTAKING;

        return Commands.parallel(
                IntakeCommands.getTargetPivotStateCommand(IntakePivotState.HOMED),
                ShooterCommands.getTargetShooterPivotCommand(ShooterPivotState.HOMED));
    }

    public Command shootingWhileMoving() {
        mRobotState = SuperStructureState.SHOOTING_MOVING;

        ShootingParameters shootingParams = ShootingCalculatorUtil.calculateShootingParameters(
                andromedaSwerve.getPose(),
                andromedaSwerve.getFieldRelativeChassisSpeeds(), andromedaSwerve.getSwerveAngle());

        this.currentParameters = shootingParams;

        return Commands.parallel(
                new SwerveDriveCommand(
                        andromedaSwerve,
                        () -> shootingParams.robotSpeed().vxMetersPerSecond,
                        () -> shootingParams.robotSpeed().vyMetersPerSecond,
                        () -> shootingParams.robotSpeed().omegaRadiansPerSecond,
                        () -> true));
    }

    public Command goToAmp() {
        return andromedaSwerve.getPathFindPath(GeomUtil.toPose2d(AllianceFlipUtil.apply(FieldConstants.ampCenter)));
    }

    public Command goToSpeaker() {
        return andromedaSwerve.getPathFindPath(GeomUtil
                .toPose2d(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d())));
    }

    public ShootingParameters getCurrentParameters() {
        return currentParameters;
    }
}

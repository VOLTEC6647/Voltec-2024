/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 3 02 2024
 */

package com.team6647.subsystems.intake;

import com.team6647.Robot;
import com.team6647.RobotContainer;
import com.team6647.subsystems.intake.IntakePivotSubsystem.IntakePivotState;
import com.team6647.subsystems.shooter.ShooterSubsystem.RollerState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class IntakeCommands {
    /*
     * private static IntakeSubsystem intakeSubsystem =
     * RobotContainer.intakeSubsystem;
     * 
     * public static final Command getTargetStateIntakeCommand(RollerState
     * desiredState) {
     * return new StartEndCommand(() ->
     * intakeSubsystem.changeRollerState(desiredState),
     * () -> intakeSubsystem.changeRollerState(RollerState.STOPPED),
     * intakeSubsystem);
     * }
     */

    private static IntakePivotSubsystem intakePivotSubsystem = RobotContainer.intakePivotSubsystem;

    public static final Command getTargetPivotStateCommand(IntakePivotState desiredState) {
        return new FunctionalCommand(
                () -> intakePivotSubsystem.changeIntakePivotState(desiredState),
                () -> {
                },
                interrupted -> {
                },
                () -> intakePivotSubsystem.inTolerance(), intakePivotSubsystem);
    }
}

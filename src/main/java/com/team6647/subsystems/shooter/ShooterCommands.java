/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 13 02 2024
 */

package com.team6647.subsystems.shooter;

import com.team6647.RobotContainer;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem.ShooterPivotState;
import com.team6647.util.Constants.RobotConstants.RollerState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ShooterCommands {
    private static ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
    private static ShooterPivotSubsystem pivotSubsystem = RobotContainer.shooterPivotSubsystem;

    public static final Command getTargetShooterPivotCommand(ShooterPivotState desiredState) {
        return new FunctionalCommand(
                () -> pivotSubsystem.setShooterPivotState(desiredState),
                () -> {
                },
                interrupted -> {
                },
                () -> pivotSubsystem.inTolerance(), pivotSubsystem);
    }

    public static final Command getTargetRollersCommand(RollerState desiredState) {
        return new StartEndCommand(() -> shooterSubsystem.changeRollerState(desiredState),
                () -> shooterSubsystem.changeRollerState(RollerState.STOPPED),
                shooterSubsystem);
    }
}

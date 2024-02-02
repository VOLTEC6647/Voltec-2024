/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 13 02 2024
 */

package com.team6647.subsystems.shooter;

import com.team6647.RobotContainer;
import com.team6647.commands.ShooterPivotTarget;
import com.team6647.commands.ShooterRollerTarget;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem.ShooterPivotState;
import com.team6647.util.Constants.RobotConstants.RollerState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterCommands {
    private static ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
    private static ShooterPivotSubsystem pivotSubsystem = RobotContainer.shooterPivotSubsystem;
    private static ShooterRollerSubsystem rollerSubsystem = RobotContainer.shooterRollerSubsystem;

    public static final Command getShooterIntakingCommand() {
        return Commands.deadline(
                Commands.waitUntil(() -> !shooterSubsystem.getBeamBrake()),
                new ShooterPivotTarget(pivotSubsystem, ShooterPivotState.INDEXING),
                new ShooterRollerTarget(rollerSubsystem, RollerState.IDLE));

    }

}

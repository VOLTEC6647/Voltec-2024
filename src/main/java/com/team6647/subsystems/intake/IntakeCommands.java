/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 3 02 2024
 */

package com.team6647.subsystems.intake;

import com.team6647.RobotContainer;
import com.team6647.commands.IntakeExtend;
import com.team6647.commands.IntakeHome;
import com.team6647.commands.IntakeRollerTarget;
import com.team6647.commands.IntakeTriggerCommand;
import com.team6647.util.Constants.RobotConstants.RollerState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {

        private static IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
        private static IntakePivotSubsystem intakePivotSubsystem = RobotContainer.intakePivotSubsystem;

        public static final Command getIntakeCommand() {

                return Commands.sequence(
                                new IntakeRollerTarget(intakeSubsystem, RollerState.INTAKING),
                                new IntakeExtend(intakePivotSubsystem),
                                new IntakeTriggerCommand(),
                                new IntakeHome(intakePivotSubsystem),
                                new IntakeRollerTarget(
                                                intakeSubsystem,
                                                RollerState.STOPPED),
                                Commands.waitSeconds(0.5),
                                new IntakeRollerTarget(
                                                intakeSubsystem,
                                                RollerState.INTAKING));
        }

        public static Command getIntakingCommandPart1() {
                return Commands.sequence(
                                new IntakeRollerTarget(intakeSubsystem, RollerState.INTAKING),
                                new IntakeExtend(intakePivotSubsystem));
        }

        public static final Command getIntakingCommandPart2() {
                return Commands.sequence(
                                new IntakeHome(intakePivotSubsystem),
                                new IntakeRollerTarget(
                                                intakeSubsystem,
                                                RollerState.STOPPED),
                                Commands.waitSeconds(0.5),
                                new IntakeRollerTarget(
                                                intakeSubsystem,
                                                RollerState.INTAKING));

        }

}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 3 02 2024
 */

package com.team6647.subsystems.intake;

import com.team6647.RobotContainer;
import com.team6647.commands.IntakePivotTarget;
import com.team6647.commands.IntakeRollerTarget;
import com.team6647.commands.IntakeTriggerCommand;
import com.team6647.subsystems.intake.IntakePivotSubsystem.IntakePivotState;
import com.team6647.util.Constants.RobotConstants.RollerState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeCommands {

        private static IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
        private static IntakePivotSubsystem intakePivotSubsystem = RobotContainer.intakePivotSubsystem;

        public static final Command getIntakeCommand() {

                return new SequentialCommandGroup(
                                new IntakeRollerTarget(intakeSubsystem, RollerState.INTAKING),
                                new IntakePivotTarget(intakePivotSubsystem, IntakePivotState.EXTENDED),
                                new IntakeTriggerCommand().withTimeout(3),
                                new IntakePivotTarget(
                                                intakePivotSubsystem,
                                                IntakePivotState.HOMED),
                                new IntakeRollerTarget(
                                                intakeSubsystem,/*  */
                                                RollerState.STOPPED),
                                Commands.waitSeconds(0.5),
                                new IntakeRollerTarget(
                                                intakeSubsystem,
                                                RollerState.INTAKING));

        }

}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 3 02 2024
 */

package com.team6647.subsystems.intake;

import com.team6647.RobotContainer;
import com.team6647.commands.IntakePivotTarget;
import com.team6647.commands.IntakeRollerStartEnd;
import com.team6647.commands.IntakeRollerTarget;
import com.team6647.subsystems.intake.IntakePivotSubsystem.IntakePivotState;
import com.team6647.util.Constants.RobotConstants.RollerState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeCommands {

        private static IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
        private static IntakePivotSubsystem intakePivotSubsystem = RobotContainer.intakePivotSubsystem;

        public static final Command getIntakeCommand() {
                Trigger beamBrakeTrigger = new Trigger(() -> intakePivotSubsystem.getBeamBrake());

                return Commands.sequence(
                                Commands.deadline(
                                                new IntakePivotTarget(intakePivotSubsystem, IntakePivotState.EXTENDED),
                                                new IntakeRollerStartEnd(intakeSubsystem, RollerState.INTAKING,
                                                                RollerState.IDLE)),
                                new RunCommand(() -> {
                                        beamBrakeTrigger
                                                        .onFalse(
                                                                        Commands.sequence(
                                                                                        new InstantCommand(
                                                                                                        () -> intakePivotSubsystem
                                                                                                                        .changeIntakePivotState(
                                                                                                                                        IntakePivotState.HOMED)),
                                                                                        new IntakeRollerTarget(
                                                                                                        intakeSubsystem,
                                                                                                        RollerState.IDLE),
                                                                                        Commands.waitSeconds(0.5),
                                                                                        new IntakeRollerTarget(
                                                                                                        intakeSubsystem,
                                                                                                        RollerState.INTAKING)));
                                }, intakePivotSubsystem, intakeSubsystem));
        }

}

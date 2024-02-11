/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 08 01 2023
 */
package com.team6647;

import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModuleIO;
import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModuleIOSim;
import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModuleIOTalonFX;
import com.andromedalib.andromedaSwerve.andromedaModule.GyroIO;
import com.andromedalib.andromedaSwerve.andromedaModule.GyroIOPigeon2;
import com.andromedalib.andromedaSwerve.commands.SwerveDriveCommand;
import com.andromedalib.andromedaSwerve.config.AndromedaModuleConfig;
import com.andromedalib.andromedaSwerve.config.AndromedaModuleConfig.AndromedaProfiles;
import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.andromedalib.andromedaSwerve.utils.AndromedaMap;
import com.andromedalib.robot.SuperRobotContainer;
import com.team6647.subsystems.elevator.ElevatorIO;
import com.team6647.subsystems.elevator.ElevatorIOSim;
import com.team6647.subsystems.elevator.ElevatorIOSparkMax;
import com.team6647.subsystems.elevator.ElevatorSubsystem;
import com.team6647.subsystems.intake.IntakeCommands;
import com.team6647.subsystems.intake.IntakeIO;
import com.team6647.subsystems.intake.IntakeIOSim;
import com.team6647.subsystems.intake.IntakeIOTalonFX;
import com.team6647.subsystems.intake.IntakePivotIO;
import com.team6647.subsystems.intake.IntakePivotIOSim;
import com.team6647.subsystems.intake.IntakePivotIOSparkMax;
import com.team6647.subsystems.intake.IntakePivotSubsystem;
import com.team6647.subsystems.intake.IntakeSubsystem;
import com.team6647.subsystems.intake.IntakePivotSubsystem.IntakePivotState;
import com.team6647.subsystems.shooter.ShooterIO;
import com.team6647.subsystems.shooter.ShooterIOSim;
import com.team6647.subsystems.shooter.ShooterIOSparkMax;
import com.team6647.subsystems.shooter.ShooterPivotIO;
import com.team6647.subsystems.shooter.ShooterPivotIOSim;
import com.team6647.subsystems.shooter.ShooterPivotIOSparkMax;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.ShooterSubsystem;
import com.team6647.subsystems.shooter.ShooterSubsystem.FlywheelState;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.OperatorConstants;
import com.team6647.util.Constants.RobotConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer extends SuperRobotContainer {
        private static RobotContainer instance;

        public static AndromedaSwerve andromedaSwerve;
        public static IntakePivotSubsystem intakePivotSubsystem;
        public static IntakeSubsystem intakeSubsystem;
        /*
         * public static ElevatorSubsystem elevatorSubsystem;
         */ public static ShooterPivotSubsystem shooterPivotSubsystem;
        public static ShooterSubsystem shooterSubsystem;
        // private VisionAutoSubsystem visionAutoSubsystem;
        // private ElevatorSubsystem elevatorSubsystem;

        private RobotContainer() {
        }

        public static RobotContainer getInstance() {
                if (instance == null) {
                        instance = new RobotContainer();
                }

                return instance;
        }

        @Override
        public void initSubsystems() {
                switch (RobotConstants.getMode()) {
                        case REAL:
                                andromedaSwerve = AndromedaSwerve.getInstance(
                                                new GyroIOPigeon2(DriveConstants.gyroID, "6647_CANivore"),
                                                new AndromedaModuleIO[] {
                                                                new AndromedaModuleIOTalonFX(0,
                                                                                AndromedaModuleConfig.getConfig(
                                                                                                AndromedaProfiles.ANDROMEDA_CONFIG,
                                                                                                AndromedaMap.mod1Const)),
                                                                new AndromedaModuleIOTalonFX(1,
                                                                                AndromedaModuleConfig.getConfig(
                                                                                                AndromedaProfiles.ANDROMEDA_CONFIG,
                                                                                                AndromedaMap.mod2Const)),
                                                                new AndromedaModuleIOTalonFX(2,
                                                                                AndromedaModuleConfig.getConfig(
                                                                                                AndromedaProfiles.ANDROMEDA_CONFIG,
                                                                                                AndromedaMap.mod3Const)),
                                                                new AndromedaModuleIOTalonFX(3,
                                                                                AndromedaModuleConfig.getConfig(
                                                                                                AndromedaProfiles.ANDROMEDA_CONFIG,
                                                                                                AndromedaMap.mod4Const)),
                                                }, DriveConstants.andromedaSwerveConfig,
                                                DriveConstants.holonomicPathConfig);
                                intakeSubsystem = IntakeSubsystem.getInstance(new IntakeIOTalonFX());
                                /*
                                 * elevatorSubsystem = ElevatorSubsystem.getInstance(new ElevatorIOSparkMax());
                                 */ intakePivotSubsystem = IntakePivotSubsystem
                                                .getInstance(new IntakePivotIOSparkMax());
                                shooterPivotSubsystem = ShooterPivotSubsystem.getInstance(new ShooterPivotIOSparkMax());
                                shooterSubsystem = ShooterSubsystem.getInstance(new ShooterIOSparkMax());
                                break;
                        case SIM:
                                andromedaSwerve = AndromedaSwerve.getInstance(new GyroIO() {
                                }, new AndromedaModuleIO[] {
                                                new AndromedaModuleIOSim(0.1),
                                                new AndromedaModuleIOSim(0.1),
                                                new AndromedaModuleIOSim(0.1),
                                                new AndromedaModuleIOSim(0.1),
                                }, DriveConstants.andromedaSwerveConfig, DriveConstants.holonomicPathConfig);
                                intakeSubsystem = IntakeSubsystem.getInstance(new IntakeIOSim());
                                /*
                                 * elevatorSubsystem = ElevatorSubsystem.getInstance(new ElevatorIOSim());
                                 */ intakePivotSubsystem = IntakePivotSubsystem.getInstance(new IntakePivotIOSim());
                                shooterPivotSubsystem = ShooterPivotSubsystem.getInstance(new ShooterPivotIOSim());
                                shooterSubsystem = ShooterSubsystem.getInstance(new ShooterIOSim());
                                break;

                        default:
                                andromedaSwerve = AndromedaSwerve.getInstance(
                                                new GyroIO() {
                                                }, new AndromedaModuleIO[] {
                                                                new AndromedaModuleIO() {
                                                                },
                                                                new AndromedaModuleIO() {
                                                                },
                                                                new AndromedaModuleIO() {
                                                                },
                                                                new AndromedaModuleIO() {
                                                                },
                                                }, DriveConstants.andromedaSwerveConfig,
                                                DriveConstants.holonomicPathConfig);
                                intakeSubsystem = IntakeSubsystem.getInstance(new IntakeIO() {
                                });
                                /*
                                 * elevatorSubsystem = ElevatorSubsystem.getInstance(new ElevatorIO() {
                                 * });
                                 */ intakePivotSubsystem = IntakePivotSubsystem.getInstance(new IntakePivotIO() {
                                });
                                shooterPivotSubsystem = ShooterPivotSubsystem.getInstance(new ShooterPivotIO() {
                                });
                                shooterSubsystem = ShooterSubsystem.getInstance(new ShooterIO() {
                                });
                                break;
                }
        }

        @Override
        public void configureBindings() {
                andromedaSwerve.setDefaultCommand(
                                new SwerveDriveCommand(
                                                andromedaSwerve,
                                                () -> -OperatorConstants.driverController1.getLeftX(),
                                                () -> -OperatorConstants.driverController1.getLeftY(),
                                                () -> -OperatorConstants.driverController1.getRightX(),
                                                () -> OperatorConstants.driverController1.leftStick().getAsBoolean()));

                /*
                 * OperatorConstants.driverController2.a().whileTrue(
                 * new InstantCommand(() ->c
                 * elevatorSubsystem.changeElevatorState(ElevatorState.TOP)));
                 * OperatorConstants.driverController2.b().whileTrue(
                 * new InstantCommand(() ->|
                 * elevatorSubsystem.changeElevatorState(ElevatorState.HOMED)));
                 */

                OperatorConstants.RUN_INTAKE_FORWARD
                                .whileTrue(IntakeCommands.getTargetStateIntakeCommand(FlywheelState.INTAKING));
                OperatorConstants.RUN_INTAKE_BACKWARD
                                .whileTrue(IntakeCommands.getTargetStateIntakeCommand(FlywheelState.EXHAUSTING));

                OperatorConstants.HOME_INTAKE
                                .whileTrue(IntakeCommands.getTargetPivotStateCommand(IntakePivotState.HOMED));
                OperatorConstants.EXTEND_INTAKE
                                .whileTrue(IntakeCommands.getTargetPivotStateCommand(IntakePivotState.EXTENDED));

        }

        public void configSysIdBindings() {
                /*
                 * OperatorConstants.FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER
                 * .whileTrue(intakePivotSubsystem.sysIdQuasistatic(Direction.kForward));
                 * OperatorConstants.BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER
                 * .whileTrue(intakePivotSubsystem.sysIdQuasistatic(Direction.kReverse));
                 * OperatorConstants.FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER
                 * .whileTrue(intakePivotSubsystem.sysIdDynamic(Direction.kForward));
                 * OperatorConstants.BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER
                 * .whileTrue(intakePivotSubsystem.sysIdDynamic(Direction.kReverse));
                 */
        }

        @Override
        public Command getAutonomousCommand() {
                return null;
        }
}

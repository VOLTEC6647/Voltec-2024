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
import com.team6647.commands.ElevatorTarget;
import com.team6647.subsystems.SuperStructure;
import com.team6647.subsystems.SuperStructure.SuperStructureState;
import com.team6647.subsystems.elevator.ElevatorIO;
import com.team6647.subsystems.elevator.ElevatorIOSim;
import com.team6647.subsystems.elevator.ElevatorIOSparkMax;
import com.team6647.subsystems.elevator.ElevatorSubsystem;
import com.team6647.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import com.team6647.subsystems.flywheel.ShooterIO;
import com.team6647.subsystems.flywheel.ShooterIOKraken;
import com.team6647.subsystems.flywheel.ShooterIOSim;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.intake.IntakeIO;
import com.team6647.subsystems.intake.IntakeIOSim;
import com.team6647.subsystems.intake.IntakeIOTalonFX;
import com.team6647.subsystems.intake.IntakePivotIO;
import com.team6647.subsystems.intake.IntakePivotIOSim;
import com.team6647.subsystems.intake.IntakePivotIOSparkMaxKraken;
import com.team6647.subsystems.intake.IntakePivotSubsystem;
import com.team6647.subsystems.intake.IntakeSubsystem;
import com.team6647.subsystems.neural.NeuralVisionIO;
import com.team6647.subsystems.neural.NeuralVisionIOLimelight;
import com.team6647.subsystems.neural.NeuralVisionSubsystem;
import com.team6647.subsystems.shooter.ShooterIORollerSim;
import com.team6647.subsystems.shooter.ShooterIORollerSparkMax;
import com.team6647.subsystems.shooter.ShooterPivotIO;
import com.team6647.subsystems.shooter.ShooterPivotIOSim;
import com.team6647.subsystems.shooter.ShooterPivotIOSparkMax;
import com.team6647.subsystems.shooter.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.ShooterRollerIO;
import com.team6647.subsystems.shooter.ShooterRollerSubsystem;
import com.team6647.subsystems.vision.VisionSubsystem;
import com.team6647.subsystems.vision.VisionIO;
import com.team6647.subsystems.vision.VisionIOLimelight;
import com.team6647.subsystems.vision.VisionIOSim;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.OperatorConstants;
import com.team6647.util.Constants.RobotConstants;
import com.team6647.util.Constants.RobotConstants.RollerState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer extends SuperRobotContainer {
        private static RobotContainer instance;

        public static AndromedaSwerve andromedaSwerve;
        public static IntakePivotSubsystem intakePivotSubsystem;
        public static IntakeSubsystem intakeSubsystem;
        public static ShooterPivotSubsystem shooterPivotSubsystem;
        public static ShooterSubsystem shooterSubsystem;
        public static ShooterRollerSubsystem shooterRollerSubsystem;
        public static VisionSubsystem visionSubsytem;
        public static NeuralVisionSubsystem neuralVisionSubsystem;
        public static ElevatorSubsystem elevatorSubsystem;

        public static SuperStructure superStructure;

        private RobotContainer() {
        }

        public static RobotContainer getInstance() {
                if (instance == null) {
                        instance = new RobotContainer();
                }
                System.out.println("Mau was here");
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
                                intakePivotSubsystem = IntakePivotSubsystem
                                                .getInstance(new IntakePivotIOSparkMaxKraken());
                                shooterPivotSubsystem = ShooterPivotSubsystem.getInstance(new ShooterPivotIOSparkMax());
                                shooterSubsystem = ShooterSubsystem.getInstance(new ShooterIOKraken());
                                shooterRollerSubsystem = ShooterRollerSubsystem
                                                .getInstance(new ShooterIORollerSparkMax());
                                elevatorSubsystem = ElevatorSubsystem.getInstance(new ElevatorIOSparkMax());
                                visionSubsytem = VisionSubsystem.getInstance(new VisionIOLimelight());
                                neuralVisionSubsystem = NeuralVisionSubsystem
                                                .getInstance(new NeuralVisionIOLimelight());
                                break;
                        case SIM:
                                andromedaSwerve = AndromedaSwerve.getInstance(
                                                new GyroIO() {
                                                }, new AndromedaModuleIO[] {
                                                                new AndromedaModuleIOSim(0.1),
                                                                new AndromedaModuleIOSim(0.1),
                                                                new AndromedaModuleIOSim(0.1),
                                                                new AndromedaModuleIOSim(0.1),
                                                }, DriveConstants.andromedaSwerveConfig,
                                                DriveConstants.holonomicPathConfig);
                                intakeSubsystem = IntakeSubsystem.getInstance(new IntakeIOSim());
                                elevatorSubsystem = ElevatorSubsystem.getInstance(new ElevatorIOSim());
                                intakePivotSubsystem = IntakePivotSubsystem.getInstance(new IntakePivotIOSim());
                                shooterPivotSubsystem = ShooterPivotSubsystem.getInstance(new ShooterPivotIOSim());
                                shooterSubsystem = ShooterSubsystem.getInstance(new ShooterIOSim());
                                shooterRollerSubsystem = ShooterRollerSubsystem.getInstance(new ShooterIORollerSim());
                                elevatorSubsystem = ElevatorSubsystem.getInstance(new ElevatorIOSim());
                                visionSubsytem = VisionSubsystem.getInstance(new VisionIOSim());
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
                                elevatorSubsystem = ElevatorSubsystem.getInstance(new ElevatorIO() {
                                });
                                intakePivotSubsystem = IntakePivotSubsystem.getInstance(new IntakePivotIO() {
                                });
                                shooterPivotSubsystem = ShooterPivotSubsystem.getInstance(new ShooterPivotIO() {
                                });
                                shooterSubsystem = ShooterSubsystem.getInstance(new ShooterIO() {
                                });
                                shooterRollerSubsystem = ShooterRollerSubsystem.getInstance(new ShooterRollerIO() {
                                });
                                elevatorSubsystem = ElevatorSubsystem.getInstance(new ElevatorIO() {
                                });
                                visionSubsytem = VisionSubsystem.getInstance(new VisionIO() {
                                });
                                neuralVisionSubsystem = NeuralVisionSubsystem.getInstance(new NeuralVisionIO() {
                                });
                                break;
                }
                superStructure = SuperStructure.getInstance();

                /*
                 * NamedCommands.registerCommand("ShootStay",
                 * SuperStructure.update(SuperStructureState.SHOOTING_SPEAKER).withTimeout(4));
                 * // TODO
                 * // QUITAR
                 * NamedCommands.registerCommand("GrabPiece",
                 * SuperStructure.update(SuperStructureState.INTAKING));
                 * NamedCommands.registerCommand("Idle",
                 * SuperStructure.update(SuperStructureState.IDLE));
                 * NamedCommands.registerCommand("VisionAlign",
                 * SuperStructure.update(SuperStructureState.INTAKE_ALIGN));
                 * 
                 * NamedCommands.registerCommand("ShootMove", Commands.waitSeconds(0));
                 */

                configSysIdBindings();
                // configTuningBindings();
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

                /* Driver 1 */

                /*
                 * OperatorConstants.GO_TO_AMP.whileTrue(SuperStructure.goToAmp());
                 */

                /* Driver 2 */
                OperatorConstants.TOGGLE_INTAKE
                                .whileTrue(SuperStructure.update(SuperStructureState.INTAKING))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                /*
                 * OperatorConstants.driverController2.povLeft()
                 * .whileTrue(ShooterCommands.getShooterIntakingCommand()
                 * .alongWith(IntakeCommands.getIntakingCommandPart1()))
                 * .onFalse(SuperStructure.update(SuperStructureState.IDLE))
                 * .and(OperatorConstants.driverController2.a())
                 * .whileTrue(IntakeCommands.getIntakingCommandPart2());
                 */

                OperatorConstants.SHOOT_SPEAKER
                                .whileTrue(SuperStructure.update(SuperStructureState.SHOOTING_SPEAKER))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                OperatorConstants.TOGGLE_AMP
                                .whileTrue(SuperStructure.update(SuperStructureState.SCORING_AMP))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                OperatorConstants.CLIMB_TOP.whileTrue(SuperStructure.update(
                                SuperStructureState.CLIMBING))
                                .onFalse(new ElevatorTarget(elevatorSubsystem, ElevatorState.HOMED));
                /*
                 * OperatorConstants.driverController2.y().whileTrue(new
                 * VisionIntakeAlign(neuralVisionSubsystem,
                 * andromedaSwerve))
                 * .onFalse(SuperStructure.update(SuperStructureState.IDLE));
                 * 
                 * OperatorConstants.driverController1.a()
                 * .whileTrue(new ShootingWhileMoving(andromedaSwerve, superStructure));
                 * 
                 * OperatorConstants.driverController2.y().whileTrue(new
                 * VisionIntakeAlign(neuralVisionSubsystem,
                 * andromedaSwerve));
                 */

        }

        public void configSysIdBindings() {

                OperatorConstants.FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER
                                .whileTrue(shooterSubsystem.sysIdQuasistatic(Direction.kForward));
                OperatorConstants.BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER
                                .whileTrue(shooterSubsystem.sysIdQuasistatic(Direction.kReverse));

                OperatorConstants.FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER
                                .whileTrue(shooterSubsystem.sysIdDynamic(Direction.kForward));
                OperatorConstants.BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER
                                .whileTrue(shooterSubsystem.sysIdDynamic(Direction.kReverse));
        }

        public void configTuningBindings() {
                OperatorConstants.driverController2.povLeft()
                                .whileTrue(new InstantCommand(
                                                () -> shooterRollerSubsystem.changeRollerState(RollerState.INTAKING)))
                                .onFalse(new InstantCommand(
                                                () -> shooterRollerSubsystem.changeRollerState(RollerState.STOPPED)));

        }

        @Override
        public Command getAutonomousCommand() {
                return Commands.waitSeconds(0);
        }
}

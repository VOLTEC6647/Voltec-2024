/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 08 01 2023
 */
package com.team6647;

import java.util.function.Function;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team6647.commands.InitIntake;
import com.team6647.commands.IntakeRollerStartEnd;
import com.team6647.commands.ShooterRollerStartEnd;
import com.team6647.commands.ShootingStationary;
import com.team6647.subsystems.SuperStructure;
import com.team6647.subsystems.SuperStructure.SuperStructureState;
import com.team6647.subsystems.flywheel.ShooterIO;
import com.team6647.subsystems.flywheel.ShooterIOKraken;
import com.team6647.subsystems.flywheel.ShooterIOSim;
import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.intake.pivot.IntakePivotIO;
import com.team6647.subsystems.intake.pivot.IntakePivotIOSim;
import com.team6647.subsystems.intake.pivot.IntakePivotIOSparkMaxKraken;
import com.team6647.subsystems.intake.pivot.IntakePivotSubsystem;
import com.team6647.subsystems.intake.roller.IntakeIO;
import com.team6647.subsystems.intake.roller.IntakeIOSim;
import com.team6647.subsystems.intake.roller.IntakeIOTalonFX;
import com.team6647.subsystems.intake.roller.IntakeSubsystem;
import com.team6647.subsystems.intake.roller.IntakeSubsystem.IntakeRollerState;
import com.team6647.subsystems.leds.LEDSubsystem;
import com.team6647.subsystems.neural.NeuralVisionIO;
import com.team6647.subsystems.neural.NeuralVisionIOLimelight;
import com.team6647.subsystems.neural.NeuralVisionSubsystem;
import com.team6647.subsystems.shooter.pivot.ShooterPivotIO;
import com.team6647.subsystems.shooter.pivot.ShooterPivotIOSim;
import com.team6647.subsystems.shooter.pivot.ShooterPivotIOSparkMax;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.roller.ShooterIORollerSim;
import com.team6647.subsystems.shooter.roller.ShooterIORollerSparkMax;
import com.team6647.subsystems.shooter.roller.ShooterRollerIO;
import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem;
import com.team6647.subsystems.shooter.roller.ShooterRollerSubsystem.ShooterFeederState;
import com.team6647.subsystems.vision.VisionSubsystem;
import com.team6647.subsystems.vision.VisionIO;
import com.team6647.subsystems.vision.VisionIOLimelight;
import com.team6647.subsystems.vision.VisionIOSim;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.OperatorConstants;
import com.team6647.util.Constants.RobotConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

        private static final LEDSubsystem leds = LEDSubsystem.getInstance();

        public static SuperStructure superStructure;

        private static LoggedDashboardChooser<Command> autoDashboardChooser;

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
                                intakePivotSubsystem = IntakePivotSubsystem.getInstance(new IntakePivotIOSim());
                                shooterPivotSubsystem = ShooterPivotSubsystem.getInstance(new ShooterPivotIOSim());
                                shooterSubsystem = ShooterSubsystem.getInstance(new ShooterIOSim());
                                shooterRollerSubsystem = ShooterRollerSubsystem.getInstance(new ShooterIORollerSim());
                                visionSubsytem = VisionSubsystem.getInstance(new VisionIOSim());
                                neuralVisionSubsystem = NeuralVisionSubsystem
                                                .getInstance(new NeuralVisionIOLimelight());
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

                                intakePivotSubsystem = IntakePivotSubsystem.getInstance(new IntakePivotIO() {
                                });
                                shooterPivotSubsystem = ShooterPivotSubsystem.getInstance(new ShooterPivotIO() {
                                });
                                shooterSubsystem = ShooterSubsystem.getInstance(new ShooterIO() {
                                });
                                shooterRollerSubsystem = ShooterRollerSubsystem.getInstance(new ShooterRollerIO() {
                                });
                                visionSubsytem = VisionSubsystem.getInstance(new VisionIO() {
                                });
                                neuralVisionSubsystem = NeuralVisionSubsystem.getInstance(new NeuralVisionIO() {
                                });
                                break;
                }
                superStructure = SuperStructure.getInstance();

                // -------- Auto Declaration --------

                NamedCommands.registerCommand("InitIntake",
                                new InitIntake(intakePivotSubsystem));
                NamedCommands.registerCommand("ShootSubwoofer",
                                SuperStructure.update(SuperStructureState.SHOOTING_SUBWOOFER).withTimeout(7));
                NamedCommands.registerCommand("ShootMiddle",
                                SuperStructure.autoMiddleCommand().withTimeout(7));
                NamedCommands.registerCommand("ShootTop",
                                SuperStructure.autoTopCommand().withTimeout(7));
                NamedCommands.registerCommand("AmpScore",
                                SuperStructure.update(SuperStructureState.AUTO_AMP));
                NamedCommands.registerCommand("ShootStay",
                                SuperStructure.update(SuperStructureState.SHOOTING_SPEAKER).withTimeout(7));
                NamedCommands.registerCommand("GrabPiece",
                                SuperStructure.update(SuperStructureState.AUTO_INTAKING));
                NamedCommands.registerCommand("Idle",
                                SuperStructure.update(SuperStructureState.AUTO_IDLE).withTimeout(1));
                NamedCommands.registerCommand("VisionAlign",
                                SuperStructure.update(SuperStructureState.INTAKE_ALIGN));
                NamedCommands.registerCommand("IntakeDown",
                                SuperStructure.update(SuperStructureState.INTAKING).withTimeout(1));

                NamedCommands.registerCommand("ShootMove", Commands.waitSeconds(0));

                autoDashboardChooser = new LoggedDashboardChooser<>("Auto chooser", AutoBuilder.buildAutoChooser());

                // -------- Engame alers (Credits: 6328) --------
                Function<Double, Command> controllerRumbleCommandFactory = time -> Commands.sequence(
                                Commands.runOnce(
                                                () -> {
                                                        OperatorConstants.driverController1.getHID()
                                                                        .setRumble(RumbleType.kBothRumble, 1.0);
                                                        OperatorConstants.driverController2.getHID()
                                                                        .setRumble(RumbleType.kBothRumble, 1.0);
                                                        leds.endgameAlert = true;
                                                }),
                                Commands.waitSeconds(time),
                                Commands.runOnce(
                                                () -> {
                                                        OperatorConstants.driverController1.getHID()
                                                                        .setRumble(RumbleType.kBothRumble, 0.0);
                                                        OperatorConstants.driverController2.getHID()
                                                                        .setRumble(RumbleType.kBothRumble, 0.0);
                                                        leds.endgameAlert = false;
                                                }));
                new Trigger(
                                () -> DriverStation.isTeleopEnabled()
                                                && DriverStation.getMatchTime() > 0
                                                && DriverStation.getMatchTime() <= Math.round(30.0))
                                .onTrue(controllerRumbleCommandFactory.apply(0.5));

                new Trigger(
                                () -> DriverStation.isTeleopEnabled()
                                                && DriverStation.getMatchTime() > 0
                                                && DriverStation.getMatchTime() <= Math.round(10.0))
                                .onTrue(
                                                Commands.sequence(
                                                                controllerRumbleCommandFactory.apply(0.2),
                                                                Commands.waitSeconds(0.1),
                                                                controllerRumbleCommandFactory.apply(0.2),
                                                                Commands.waitSeconds(0.1),
                                                                controllerRumbleCommandFactory.apply(0.2)));

                // configSysIdBindings();
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

                // -------- Gyro Commands --------

                OperatorConstants.RESET_GYRO
                                .whileTrue(new InstantCommand(() -> andromedaSwerve.setGyroAngle(new Rotation2d())));

                /* Driver 2 */

                OperatorConstants.FORCE_IDLE
                                .whileTrue(SuperStructure.update(SuperStructureState.IDLE));

                // -------- Superstructure --------

                // -------- Intake Commands --------

                OperatorConstants.TOGGLE_INTAKE
                                .whileTrue(SuperStructure.update(SuperStructureState.INTAKING))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // -------- Shooter Commands --------

                OperatorConstants.SHOOT_SPEAKER
                                .whileTrue(SuperStructure.update(SuperStructureState.SHOOTING_SPEAKER))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // Subwoofer shootings
                OperatorConstants.SHOOT_SUBWOOFER
                                .whileTrue(SuperStructure.update(SuperStructureState.SHOOTING_SUBWOOFER))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // Shooting with feeder detection

                OperatorConstants.INTELLIGENT_SHOOTING
                                .whileTrue(SuperStructure.update(SuperStructureState.INTELLIGENT_SHOOTING_SPEAKER))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // -------- Amp Commands --------

                OperatorConstants.TOGGLE_AMP
                                .whileTrue(SuperStructure.update(SuperStructureState.SCORING_AMP))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // -------- Helper Commands --------

                OperatorConstants.INTAKE_FEEDER
                                .whileTrue(Commands.parallel(
                                                new ShooterRollerStartEnd(shooterRollerSubsystem,
                                                                ShooterFeederState.INTAKING,
                                                                ShooterFeederState.STOPPED),
                                                new IntakeRollerStartEnd(intakeSubsystem, IntakeRollerState.INTAKING,
                                                                IntakeRollerState.STOPPED)));

                OperatorConstants.EXHAUST_FEEDER
                                .whileTrue(Commands.parallel(
                                                new ShooterRollerStartEnd(shooterRollerSubsystem,
                                                                ShooterFeederState.EXHAUSTING,
                                                                ShooterFeederState.STOPPED),
                                                new IntakeRollerStartEnd(intakeSubsystem, IntakeRollerState.EXHAUSTING,
                                                                IntakeRollerState.STOPPED)));
        }

        public void configSysIdBindings() {
                OperatorConstants.FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER
                                .whileTrue(andromedaSwerve.sysIdQuasistatic(Direction.kForward));
                OperatorConstants.BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER
                                .whileTrue(andromedaSwerve.sysIdQuasistatic(Direction.kReverse));

                OperatorConstants.FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER
                                .whileTrue(andromedaSwerve.sysIdDynamic(Direction.kForward));
                OperatorConstants.BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER
                                .whileTrue(andromedaSwerve.sysIdDynamic(Direction.kReverse));
        }

        public void configTuningBindings() {
                OperatorConstants.driverController2.povLeft()
                                .whileTrue(new InstantCommand(
                                                () -> shooterRollerSubsystem
                                                                .setMRollerState(ShooterFeederState.INTAKING)))
                                .onFalse(new InstantCommand(
                                                () -> shooterRollerSubsystem
                                                                .setMRollerState(ShooterFeederState.STOPPED)));

        }

        @Override
        public Command getAutonomousCommand() {
                return autoDashboardChooser.get();
        }
}

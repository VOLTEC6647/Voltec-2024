/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 08 01 2023
 */
package com.team6647;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Function;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModuleIO;
import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModuleIOSim;
import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModuleIOTalonFX;
import com.andromedalib.andromedaSwerve.andromedaModule.GyroIO;
import com.andromedalib.andromedaSwerve.andromedaModule.GyroIOPigeon2;
import com.andromedalib.andromedaSwerve.utils.AndromedaMap;
import com.andromedalib.robot.SuperRobotContainer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team6647.commands.InitIntake;
import com.team6647.commands.IntakeRollerStartEnd;
import com.team6647.commands.ShooterRollerStartEnd;
import com.team6647.subsystems.SuperStructure;
import com.team6647.subsystems.SuperStructure.SuperStructureState;
import com.team6647.subsystems.drive.Drive;
import com.team6647.subsystems.drive.Drive.DriveMode;
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
import com.team6647.subsystems.shooter.pivot.ShooterPivotIOTalonFX;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem;
import com.team6647.subsystems.shooter.pivot.ShooterPivotSubsystem.ShooterPivotState;
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

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer extends SuperRobotContainer {
        private static RobotContainer instance;

        public static Drive andromedaSwerve;
        public static IntakePivotSubsystem intakePivotSubsystem;
        public static IntakeSubsystem intakeSubsystem;
        public static ShooterPivotSubsystem shooterPivotSubsystem;
        public static ShooterSubsystem shooterSubsystem;
        public static ShooterRollerSubsystem shooterRollerSubsystem;
        public static VisionSubsystem visionSubsytem;
        public static NeuralVisionSubsystem neuralVisionSubsystem;
        public static RobotState robotState;

        private static final LEDSubsystem leds = LEDSubsystem.getInstance();

        public static SuperStructure superStructure;

        private static LoggedDashboardChooser<Command> autoDashboardChooser;

        SlewRateLimiter xLimiter, yLimiter, turningLimiter;

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
                                andromedaSwerve = Drive.getInstance(
                                                new GyroIOPigeon2(DriveConstants.gyroID, "6647_CANivore"),
                                                new AndromedaModuleIO[] {
                                                                new AndromedaModuleIOTalonFX(0,
                                                                                DriveConstants.andromedModuleConfig(
                                                                                                AndromedaMap.mod1Const)),
                                                                new AndromedaModuleIOTalonFX(1,
                                                                                DriveConstants.andromedModuleConfig(
                                                                                                AndromedaMap.mod2Const)),
                                                                new AndromedaModuleIOTalonFX(2,
                                                                                DriveConstants.andromedModuleConfig(
                                                                                                AndromedaMap.mod3Const)),
                                                                new AndromedaModuleIOTalonFX(3,
                                                                                DriveConstants.andromedModuleConfig(
                                                                                                AndromedaMap.mod4Const)),
                                                }, DriveConstants.andromedaSwerveConfig);
                                intakeSubsystem = IntakeSubsystem.getInstance(new IntakeIOTalonFX());
                                intakePivotSubsystem = IntakePivotSubsystem
                                                .getInstance(new IntakePivotIOSparkMaxKraken());
                                shooterPivotSubsystem = ShooterPivotSubsystem.getInstance(new ShooterPivotIOTalonFX());
                                shooterSubsystem = ShooterSubsystem.getInstance(new ShooterIOKraken());
                                shooterRollerSubsystem = ShooterRollerSubsystem
                                                .getInstance(new ShooterIORollerSparkMax());
                                visionSubsytem = VisionSubsystem.getInstance(new VisionIOLimelight());
                                neuralVisionSubsystem = NeuralVisionSubsystem
                                                .getInstance(new NeuralVisionIOLimelight());
                                break;
                        case SIM:
                                andromedaSwerve = Drive.getInstance(
                                                new GyroIO() {
                                                }, new AndromedaModuleIO[] {
                                                                new AndromedaModuleIOSim(0.1),
                                                                new AndromedaModuleIOSim(0.1),
                                                                new AndromedaModuleIOSim(0.1),
                                                                new AndromedaModuleIOSim(0.1),
                                                }, DriveConstants.andromedaSwerveConfig);
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
                                andromedaSwerve = Drive.getInstance(
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
                                                }, DriveConstants.andromedaSwerveConfig);
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
                robotState = RobotState.getInstance();

                // -------- Auto Declaration --------

                NamedCommands.registerCommand("InitIntake",
                                new InitIntake(intakePivotSubsystem));
                NamedCommands.registerCommand("ShootSubwoofer",
                                SuperStructure.update(SuperStructureState.AUTO_SHOOTING_SUBWOOFER));
                NamedCommands.registerCommand("ShootMiddle",
                                SuperStructure.autoMiddleCommand().withTimeout(7));
                NamedCommands.registerCommand("ShootTop",
                                SuperStructure.autoTopCommand().withTimeout(7));
                NamedCommands.registerCommand("AmpScore",
                                SuperStructure.update(SuperStructureState.AUTO_AMP));
                NamedCommands.registerCommand("ShootStay",
                                SuperStructure.update(SuperStructureState.AUTO_SHOOTING_SPEAKER));
                NamedCommands.registerCommand("SecondaryShootStay",
                                SuperStructure.update(SuperStructureState.SECONDARY_AUTO_SHOOTING_SPEAKER)
                                                .withTimeout(6));
                NamedCommands.registerCommand("GrabPiece",
                                SuperStructure.update(SuperStructureState.AUTO_INTAKING_COMPLETE));
                NamedCommands.registerCommand("ExtendIntake",
                                SuperStructure.update(SuperStructureState.AUTO_INTAKING));
                NamedCommands.registerCommand("IndexPiece",
                                SuperStructure.update(SuperStructureState.AUTO_INDEXING));
                NamedCommands.registerCommand("EnableNeural",
                                SuperStructure.update(SuperStructureState.ENABLE_NEURAL));
                NamedCommands.registerCommand("Idle",
                                SuperStructure.update(SuperStructureState.AUTO_IDLE).withTimeout(1));
                NamedCommands.registerCommand("VisionAlign",
                                SuperStructure.update(SuperStructureState.INTAKE_ALIGN));
                NamedCommands.registerCommand("SuppIndex",
                                SuperStructure.update(SuperStructureState.SUPP_INDEXING));

                NamedCommands.registerCommand("ShootMove", Commands.waitSeconds(0));


                autoDashboardChooser = new LoggedDashboardChooser<>("Auto chooser",
                                AutoBuilder.buildAutoChooser());

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
                                                && DriverStation.getMatchTime() <= Math.round(25.0))
                                .onTrue(Commands.sequence(
                                                controllerRumbleCommandFactory.apply(1.0),
                                                Commands.waitSeconds(0.1),
                                                controllerRumbleCommandFactory.apply(1.0)));
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
                new Trigger(() -> DriverStation.isTeleopEnabled() && !shooterSubsystem.getBeamBrake())
                                .onTrue(Commands.sequence(
                                                controllerRumbleCommandFactory.apply(0.4),
                                                Commands.waitSeconds(0.1),
                                                controllerRumbleCommandFactory.apply(0.4)

                                ));
                new Trigger(() -> shooterPivotSubsystem.getMState() != ShooterPivotState.HOMED).whileTrue(
                                Commands.sequence(
                                                new RunCommand(() -> leds.strobRed(0.2)).withTimeout(2),
                                                new RunCommand(() -> leds.solidRed())).ignoringDisable(true)
                                                .repeatedly());

                new Trigger(() -> !intakeSubsystem.getBeamBrake())
                                .whileTrue(Commands.sequence(
                                                new RunCommand(() -> leds.strobeYellow(0.2)).withTimeout(2),
                                                new RunCommand(() -> leds.solidYellow())).ignoringDisable(true)
                                                .repeatedly());

                new Trigger(() -> !shooterSubsystem.getBeamBrake())
                                .whileTrue(Commands.sequence(
                                                new RunCommand(() -> leds.strobeGreen(0.2)).withTimeout(2),
                                                new RunCommand(() -> leds.solidGreen())).ignoringDisable(true)
                                                .repeatedly());

                new Trigger(() -> shooterSubsystem.getBeamBrake() && intakeSubsystem.getBeamBrake()
                                && shooterPivotSubsystem.getMState() == ShooterPivotState.HOMED)
                                .whileTrue(Commands.sequence(
                                                new RunCommand(() -> leds.solidBlue())).ignoringDisable(true)
                                                .repeatedly());

                /*
                 * new Trigger(() -> DriverStation.isDisabled()).whileTrue(
                 * new RunCommand(() -> leds.rainbow()).ignoringDisable(true));
                 * 
                 * new Trigger(() -> DriverStation.isEnabled()).whileTrue(
                 * new RunCommand(() -> leds.solidBlue()).ignoringDisable(true));
                 */
                configSysIdBindings();
        }

        @Override
        public void configureBindings() {
                xLimiter = new SlewRateLimiter(andromedaSwerve.andromedaProfile.maxAcceleration);
                yLimiter = new SlewRateLimiter(andromedaSwerve.andromedaProfile.maxAcceleration);
                turningLimiter = new SlewRateLimiter(andromedaSwerve.andromedaProfile.maxAngularAcceleration);

                andromedaSwerve.setDefaultCommand(
                                andromedaSwerve.run(
                                                () -> {
                                                        double ySpeed = yLimiter
                                                                        .calculate(-OperatorConstants.driverController1
                                                                                        .getLeftY());
                                                        double xSpeed = xLimiter
                                                                        .calculate(-OperatorConstants.driverController1
                                                                                        .getLeftX());
                                                        double rotationSpeed = turningLimiter
                                                                        .calculate(-OperatorConstants.driverController1
                                                                                        .getRightX());

                                                        andromedaSwerve.acceptTeleopInputs(
                                                                        () -> xSpeed,
                                                                        () -> ySpeed,
                                                                        () -> rotationSpeed,
                                                                        () -> !OperatorConstants.driverController1
                                                                                        .leftStick()
                                                                                        .getAsBoolean());

                                                }));

                /* Driver 1 */

                // -------- Gyro Commands --------

                // configSysIdBindings();

                OperatorConstants.RESET_GYRO
                                .whileTrue(new InstantCommand(
                                                () -> andromedaSwerve.setGyroAngle(Rotations.of(0))));

                OperatorConstants.GO_TO_AMP.whileTrue(SuperStructure.goToAmp())
                                .onFalse(new InstantCommand(() -> Drive.setMDriveMode(DriveMode.TELEOP))
                                                .andThen(SuperStructure.update(SuperStructureState.IDLE)));

                OperatorConstants.driverController1.y()
                                .whileTrue(SuperStructure.update(SuperStructureState.SHUTTLE_ALIGN))
                                .onFalse(new InstantCommand(() -> Drive.setMDriveMode(DriveMode.TELEOP))
                                                .andThen(SuperStructure.update(SuperStructureState.IDLE)));

                /* Driver 2 */

                //OperatorConstants.FORCE_IDLE
                                //.whileTrue(SuperStructure.update(SuperStructureState.IDLE))
                                //.onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // -------- Superstructure --------

                // -------- Intake Commands --------

                // Complete intaking sequence
                OperatorConstants.TOGGLE_INTAKE
                                .whileTrue(SuperStructure.update(SuperStructureState.INTAKING_COMPLETE))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // Pass intake from intake to shooter
                OperatorConstants.INDEXING
                                .whileTrue(SuperStructure.update(SuperStructureState.INDEXING))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // Intake only, no shooter
                OperatorConstants.INTAKING_ONLY
                                .whileTrue(SuperStructure.update(SuperStructureState.INTAKING))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // Ignores Beam Break

                OperatorConstants.INTAKING_ONLY_FORCED
                                .whileTrue(SuperStructure.update(SuperStructureState.INTAKING_FORCED))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // -------- Shooter Commands --------

                OperatorConstants.PREPARE_SHOOTER
                                .onTrue(SuperStructure.update(SuperStructureState.PREPARING_SHOOTER));

                OperatorConstants.SHOOT_SPEAKER
                                .whileTrue(SuperStructure.update(SuperStructureState.SHOOTING_SPEAKER))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // Subwoofer shootings
                OperatorConstants.SHOOT_SUBWOOFER
                                .whileTrue(SuperStructure.update(SuperStructureState.SHOOTING_SUBWOOFER))
                                .onFalse(SuperStructure.update(SuperStructureState.IDLE));

                // Shooting notes to wing

                OperatorConstants.SHUTTLE
                                .whileTrue(SuperStructure.update(SuperStructureState.SHUTTLE))
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

                // -------- Climbing --------

                OperatorConstants.PREPARE_CLIMB
                                .whileTrue(SuperStructure.update(SuperStructureState.PREPARE_CLIMB))
                                .and(OperatorConstants.CLIMB
                                                .whileTrue(SuperStructure.update(SuperStructureState.CLIMBING)));
                // -------- Re enabling pivot --------

        }

        public void configSysIdBindings() {
                // OperatorConstants.FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER
                // .whileTrue(shooterPivotSubsystem.sysIdQuasistatic(Direction.kForward));
                // OperatorConstants.BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER
                // .whileTrue(shooterPivotSubsystem.sysIdQuasistatic(Direction.kReverse));

                // OperatorConstants.FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER
                // .whileTrue(shooterPivotSubsystem.sysIdDynamic(Direction.kForward));
                // OperatorConstants.BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER
                // .whileTrue(shooterPivotSubsystem.sysIdDynamic(Direction.kReverse));
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

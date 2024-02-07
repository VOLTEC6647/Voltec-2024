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
import com.team6647.subsystems.intake.IntakeCommands;
import com.team6647.subsystems.intake.IntakeIOSim;
import com.team6647.subsystems.intake.IntakeIOTalonFX;
import com.team6647.subsystems.intake.IntakePivotIOSim;
import com.team6647.subsystems.intake.IntakePivotIOSparkMax;
import com.team6647.subsystems.intake.IntakePivotSubsystem;
import com.team6647.subsystems.intake.IntakeSubsystem;
import com.team6647.subsystems.intake.IntakePivotSubsystem.IntakePivotState;
import com.team6647.subsystems.shooter.ShooterSubsystem.RollerState;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.OperatorConstants;
import com.team6647.util.Constants.RobotConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer extends SuperRobotContainer {
        private static RobotContainer instance;

        public static AndromedaSwerve andromedaSwerve;
        public static IntakePivotSubsystem intakePivotSubsystem;
        // public static IntakeSubsystem intakeSubsystem;
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
                switch (RobotConstants.currentMode) {
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
                                intakePivotSubsystem = IntakePivotSubsystem.getInstance(new IntakePivotIOSparkMax());
                                /*
                                 * visionAutoSubsystem = VisionAutoSubsystem.getInstance(
                                 * new VisionIOLimelight(Alliance.Blue),
                                 * andromedaSwerve);
                                 */
                                break;
                        case SIM:
                                andromedaSwerve = AndromedaSwerve.getInstance(new GyroIO() {
                                }, new AndromedaModuleIO[] {
                                                new AndromedaModuleIOSim(0.1),
                                                new AndromedaModuleIOSim(0.1),
                                                new AndromedaModuleIOSim(0.1),
                                                new AndromedaModuleIOSim(0.1),
                                }, DriveConstants.andromedaSwerveConfig, DriveConstants.holonomicPathConfig);
                                intakePivotSubsystem = IntakePivotSubsystem.getInstance(new IntakePivotIOSim());
                                /*
                                 * visionAutoSubsystem = VisionAutoSubsystem.getInstance(
                                 * new VisionIOLimelight(DriverStation.getAlliance().get()),
                                 * andromedaSwerve);
                                 */
                                break;

                        default:
                                andromedaSwerve = AndromedaSwerve.getInstance(
                                                new GyroIOPigeon2(DriveConstants.gyroID, "6647_CANivore") {
                                                }, new AndromedaModuleIO[] {
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
                                intakePivotSubsystem = IntakePivotSubsystem.getInstance(new IntakePivotIOSparkMax());
                                /*
                                 * visionAutoSubsystem = VisionAutoSubsystem.getInstance(
                                 * new VisionIOLimelight(DriverStation.getAlliance().get()),
                                 * andromedaSwerve);
                                 */
                                break;
                }

                // elevatorSubsystem = ElevatorSubsystem.getInstance(new ElevatorIOSim());
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

                /*
                 * OperatorConstants.RUN_INTAKE_FORWARD
                 * .whileTrue(IntakeCommands.getTargetStateIntakeCommand(RollerState.INTAKING));
                 * OperatorConstants.RUN_INTAKE_BACKWARD
                 * .whileTrue(IntakeCommands.getTargetStateIntakeCommand(RollerState.EXHAUSTING)
                 * );
                 */

                OperatorConstants.HOME_INTAKE
                                .whileTrue(IntakeCommands.getTargetPivotStateCommand(IntakePivotState.HOMED));
                OperatorConstants.EXTEND_INTAKE
                                .whileTrue(IntakeCommands.getTargetPivotStateCommand(IntakePivotState.EXTENDED));

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

        @Override
        public Command getAutonomousCommand() {
                return null;
        }
}

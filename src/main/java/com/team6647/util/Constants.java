/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.util;

import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig;
import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig.Mode;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Constants {

        public static class OperatorConstants {
                public static final int kDriverControllerPort = 0;
                public static final int kDriverControllerPort2 = 1;

                public static final CommandXboxController driverController1 = new CommandXboxController(
                                OperatorConstants.kDriverControllerPort);
                public static final CommandXboxController driverController2 = new CommandXboxController(
                                OperatorConstants.kDriverControllerPort2);

                /* Driver 1 */
                public static final Trigger FORWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = driverController1.y(),
                                BACKWARD_QUASISTATIC_CHARACTERIZATION_TRIGGER = driverController1.a(),
                                FORWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = driverController1.b(),
                                BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = driverController1.x();

                /* Driver 2 */

                public static final Trigger RUN_INTAKE_FORWARD = driverController2.rightTrigger(),
                                RUN_INTAKE_BACKWARD = driverController2.leftTrigger(),
                                TOGGLE_INTAKE = driverController2.povRight(),
                                EXTEND_INTAKE = driverController2.povLeft();

        }

        public static class RobotConstants {
                public static final Mode currentMode = Mode.REAL;

                public static Mode getMode() {
                        if ((currentMode == Mode.SIM || currentMode == Mode.REPLAY) && RobotBase.isReal()) {
                                DriverStation.reportError("[Mode Error] Mode is set to " + currentMode.toString()
                                                + " , but the robot is running on a real robot. Changing mode to avoid issues",
                                                true);
                        }

                        if (currentMode == Mode.SIM)
                                return RobotBase.isReal() ? Mode.REAL : Mode.SIM;

                        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

                }

                public enum RollerState {
                        STOPPED,
                        EXHAUSTING,
                        INTAKING,
                        IDLE
                }
        }

        public static class ShooterConstants {
                public static final int shooterPivotMotorID = 20;
                public static final int flywheelBottomMotorID = 21;
                public static final int flywheelTopMotorID = 22;
                public static final int shooterRollerMotorID = 23;

                public static final double shooterKp = 0;
                public static final double shooterKi = 0;
                public static final double shooterKd = 0;
                public static final double shooterFF = 0.0;

                public static final double shooterTolerance = 0.0;

                public static final double pivotKp = 0.028;
                public static final double piovtKi = 0.000029;
                public static final double pivotKd = 0.0;

                public static final double pivotMaxVelocity = 10.0;
                public static final double pivotMaxAcceleration = 10.0;

                public static final int shooterBeamBrakeChannel = 8;
                public static final int shooterMotorCurrentLimit = 80;
                public static final int rollerMotorCurrentLimit = 80;

                public static final double armEncoderPositionConversionFactor = 360;
                public static final double armEncoderZeroOffset = 320;
                public static final boolean armEncoderInverted = true;
                public static final boolean shooterPivotMotorInverted = true;

                public static final double pivotMinPosition = 113;
                public static final double pivotMaxPosition = 257.750;
                public static final double pivotHomedPosition = 114;
                public static final double pivotIndexingPosition = 150;

                public static final double rollerIntakingVelocity = 0.25;
                public static final double rollerExhaustingVelocity = -0.25;
                public static final double rollerIdleVelocity = 0.1;
        }

        public static class ElevatorConstants {
                public static final int elevatorBoottomMotorID = 17;
                public static final int elevatorTopMotorID = 18;

                public static final int elevatorCANCoderID = 19;

                public static final double elevatorCANCoderOffset = 0.0;

                public static final boolean elevatorTopMotorInverted = false;
                public static final boolean elevatorBottomMotorInverted = true;
                public static final int elevatorMotorCurrentLimit = 80;
                public static final double elevatorGearRatio = 24.6;

                public static final double elevatorKp = 0.01;
                public static final double elevatorKi = 0;
                public static final double elevatorKd = 0.00000000000000001;

                public static final double elevatorMinPosition = 0.64;
                public static final double elevatorMaxPosition = 0.8;
                public static final double elevatorShootingPosition = 0.70;

                public static final SensorDirectionValue elevatorCANCoderSensorValue = SensorDirectionValue.CounterClockwise_Positive;
        }

        public static class IntakeConstants {
                public static final int intakeMotorID = 16;

                public static final double homedKp = 0.004;
                public static final double homedKi = 0.00004;
                public static final double homedKd = 0.0;

                public static final double extendedKp = 0.005;
                public static final double extendedKi = 0.0;
                public static final double extendedKd = 0.0;

                public static final double extendedPIDMaxVelocity = 4500.0;
                public static final double extendedPIDMaxAcceleration = 4500.0;

                public static final double homedPIDMaxVelocity = 2500.0;
                public static final double homedPIDMaxAcceleration = 1500.0;

                public static final double minIntakePivotPosition = 137.400;
                public static final double maxIntakePivotPosition = 197.040;
                public static final double intakePivotPositionTolerance = 0.4;
                public static final double intakeHomedPosition = 136.342;
                public static final double intakeExtendedPosition = 197.000;

                public static final int intakePivotLeftMotorID = 14;
                public static final int intakePivotRightMotorID = 15;

                public static final double intakePivotEncoderPositionConversionFactor = 360;
                public static final double intakePivotEncoderZeroOffset = 0.0;

                public static final boolean intakePivotEncoderInverted = true;
                public static final boolean intakePivotLeftMotorInverted = false;
                public static final boolean intakePivotRightMotorInverted = true;

                public static final int intakeMotorsCurrentLimit = 80;

                public static final double intakeStoppedVelocity = 0.0;
                public static final double intakeIntakingVelocity = -0.5;
                public static final double intakeExhaustingVelocity = 0.5;
                public static final double intakeIdleVelocity = 0.1;

                public static final int intakeBeamBrakeChannel = 9;

        }

        public static class DriveConstants {
                public static final double trackWidth = Units.inchesToMeters(18.5);
                public static final double wheelBase = Units.inchesToMeters(18.5);
                public static final double wheelDiameter = Units.inchesToMeters(4.0);
                public static final double wheelCircumference = wheelDiameter * Math.PI;

                /*
                 * This has to do with the robot-centric coordinate system in WPILib
                 * The convention is that +x is out front from the robot’s perspective and +y is
                 * out left of the robot (you can verify this with the right-hand rule, +z is up
                 * so counter-clockwise rotation is positive, which checks out).
                 */
                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0));

                public static final double maxSpeed = 4.641013629172587;
                public static final double maxAcceleration = maxSpeed * 0.85; // 13.804786438404268;
                /** Radians per Second */
                public static final double maxAngularVelocity = 11.5; // 13.804786438404268;
                public static final double maxAngularAcceleration = 3.5; // 0.09714;

                public static final AndromedaSwerveConfig andromedaSwerveConfig = new AndromedaSwerveConfig(0.1,
                                trackWidth,
                                wheelBase, swerveKinematics, maxSpeed, maxAcceleration, maxAngularVelocity,
                                maxAngularAcceleration, wheelDiameter);

                public static final int gyroID = 13;

                /* Auto constants */
                public static final double translationP = 7;
                public static final double translationI = 0.00;
                public static final double translationD = 0.195;
                public static final double rotationP = 1;
                public static final double rotationI = 0.0;
                public static final double rotationD = 0.001;

                public static final PIDConstants translationConstants = new PIDConstants(translationP, translationI,
                                translationD);
                public static final PIDConstants rotationConstants = new PIDConstants(rotationP, rotationI,
                                rotationD);
                public static final HolonomicPathFollowerConfig holonomicPathConfig = new HolonomicPathFollowerConfig(
                                DriveConstants.translationConstants,
                                DriveConstants.rotationConstants,
                                maxSpeed,
                                Math.sqrt(Math.pow(trackWidth, 2) + Math.pow(trackWidth, 2)),
                                new ReplanningConfig());
        }
}

/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.util;

import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig;
import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig.Mode;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
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
                                HOME_INTAKE = driverController2.povRight(),
                                EXTEND_INTAKE = driverController2.povLeft();

        }

        public static class RobotConstants {
                public static final Mode currentMode = Mode.SIM;
        }

        public static class ShooterConstants {

                public static final int shooterPivotMotorID = 18;
                public static final int shooterMotorLeftID = 19;
                public static final int shooterMotorRightID = 20;

                public static final double shooterKp = 0;
                public static final double shooterKi = 0;
                public static final double shooterKd = 0;

                public static final double shooterSpeed = 0.25;
                public static final double passiveStopped = 0.1;
                public static final int beamBrakePort = 1;
                public static final int shooterMotorCurrentLimit = 80;

                public static final double armEncoderPositionConversionFactor = 360;
                public static final double armEncoderZeroOffset = 0;
                public static final boolean armEncoderInverted = false;
                public static final boolean shooterPivotMotorInverted = false;

                public static final double shooterMinPosition = 0.0;
                public static final double shooterMaxPosition = 0.0;
                public static final double shooterHomedPosition = 0.0;
        }

        public static class ElevatorConstants {
                public static final int elevatorMotorID = 13;
                public static final boolean elevatorMotorInverted = false;
                public static final int elevatorMotorCurrentLimit = 80;

                public static final double elevatorAbsoluteEncoderPositionConversionFactor = 0.0;
                public static final double elevatorAbsoluteEncoderZeroOffset = 0.0;
                public static final boolean elevatorAbsoluteEncoderInverted = false;

                public static final double elevatorKp = 0.01;
                public static final double elevatorKi = 0;
                public static final double elevatorKd = 0.00000000000000001;

                public static final double elevatorMinPosition = 0.64;
                public static final double elevatorMaxPosition = 0.8;
                public static final double elevatorShootingPosition = 0.70;
        }

        public static class IntakeConstants {
                public static final int intakeMotorID = 13;

                public static final double pivotKp = 0.003;
                public static final double pivotKi = 0.0;
                public static final double pivotKd = 0.0;

                public static final double intakePIDMaxVelocity = 1.0;
                public static final double intakePIDMaxAcceleration = 1.0;

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
                public static final double intakeIntakingVelocity = 0.5;
                public static final double intakeExhaustingVelocity = -0.5;
                public static final double intakeIdleVelocity = 0.1;

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

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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
                                BACKWARD_DYNAMIC_CHARACTERIZATION_TRIGGER = driverController1.x(),
                                RESET_GYRO = driverController1.a(),
                                GO_TO_AMP = driverController1.b(),
                                GO_TO_SPEAKER = driverController1.y();

                /* Driver 2 */

                public static final Trigger TOGGLE_INTAKE = driverController2.povRight(),
                                TOGGLE_AMP = driverController2.x(),
                                SHOOT_SPEAKER = driverController2.b(),
                                CLIMB_TOP = driverController2.povUp(),
                                INTAKE_FEEDER = driverController2.rightTrigger(),
                                EXHAUST_FEEDER = driverController2.leftTrigger();
        }

        public static class RobotConstants {
                public static final Mode currentMode = Mode.REAL;

                public static final boolean tuningMode = true;

                public static Mode getMode() {
                        if ((currentMode == Mode.SIM || currentMode == Mode.REPLAY) && RobotBase.isReal()) {
                                DriverStation.reportError("[Mode Error] Mode is set to " + currentMode.toString()
                                                + " , but the robot is running on a real robot. Changing mode to avoid issues",
                                                true);
                                return Mode.REAL;
                        }

                        if (currentMode == Mode.SIM)
                                return RobotBase.isReal() ? Mode.REAL : Mode.SIM;

                        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

                }

        }

        /**
         * Contains various field dimensions and useful reference points. Dimensions are
         * in meters, and sets
         * of corners start in the lower left moving clockwise. <b>All units in
         * Meters</b> <br>
         * <br>
         *
         * <p>
         * All translations and poses are stored with the origin at the rightmost point
         * on the BLUE
         * ALLIANCE wall.<br>
         * <br>
         * Length refers to the <i>x</i> direction (as described by wpilib) <br>
         * Width refers to the <i>y</i> direction (as described by wpilib)
         */
        public static class FieldConstants {
                public static double fieldLength = Units.inchesToMeters(651.223);
                public static double fieldWidth = Units.inchesToMeters(323.277);
                public static double wingX = Units.inchesToMeters(229.201);
                public static double podiumX = Units.inchesToMeters(126.75);
                public static double startingLineX = Units.inchesToMeters(74.111);

                public static Pose2d amp = new Pose2d(new Translation2d(1.80, 7.62), Rotation2d.fromDegrees(-90));

                /** Staging locations for each note */
                public static final class StagingLocations {
                        public static double centerlineX = fieldLength / 2.0;

                        // need to update
                        public static double centerlineFirstY = Units.inchesToMeters(29.638);
                        public static double centerlineSeparationY = Units.inchesToMeters(66);
                        public static double spikeX = Units.inchesToMeters(114);
                        // need
                        public static double spikeFirstY = Units.inchesToMeters(161.638);
                        public static double spikeSeparationY = Units.inchesToMeters(57);

                        public static Translation2d[] centerlineTranslations = new Translation2d[5];
                        public static Translation2d[] spikeTranslations = new Translation2d[3];

                        static {
                                for (int i = 0; i < centerlineTranslations.length; i++) {
                                        centerlineTranslations[i] = new Translation2d(centerlineX,
                                                        centerlineFirstY + (i * centerlineSeparationY));
                                }
                        }

                        static {
                                for (int i = 0; i < spikeTranslations.length; i++) {
                                        spikeTranslations[i] = new Translation2d(spikeX,
                                                        spikeFirstY + (i * spikeSeparationY));
                                }
                        }
                }

                /** Each corner of the speaker * */
                public static final class Speaker {

                        // corners (blue alliance origin)
                        public static Translation3d topRightSpeaker = new Translation3d(
                                        Units.inchesToMeters(18.055),
                                        Units.inchesToMeters(238.815),
                                        Units.inchesToMeters(83.091));

                        public static Translation3d topLeftSpeaker = new Translation3d(
                                        Units.inchesToMeters(18.055),
                                        Units.inchesToMeters(197.765),
                                        Units.inchesToMeters(83.091));

                        public static Translation3d bottomRightSpeaker = new Translation3d(0.0,
                                        Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
                        public static Translation3d bottomLeftSpeaker = new Translation3d(0.0,
                                        Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

                        /** Center of the speaker opening (blue alliance) */
                        public static Translation3d centerSpeakerOpening = bottomLeftSpeaker
                                        .interpolate(topRightSpeaker, 0.5);

                        public static Translation2d TargetLocation = new Translation2d(0.06, 5.55);

                }

        }

        public static class ShooterConstants {
                public static final int shooterPivotMotorID = 21;
                public static final int flywheelBottomMotorID = 22;
                public static final int flywheelTopMotorID = 23;
                public static final int shooterRollerMotorID = 24;

                public static final double bottomShooterKp = 0.44998;
                public static final double bottomShooterKi = 0.001;
                public static final double bottomShooterKd = 0.0;
                public static final double bottomShooterKs = 0.14004;
                public static final double bottomShooterKv = 0.11746;
                public static final double bottomShooterKa = 0.0029709;

                public static final double bottomShooterKf = 0.000195;

                public static final double topShooterKp = 0.46848;
                public static final double topShooterKi = 0.001;
                public static final double topShooterKd = 0.0;
                public static final double topShooterKs = 0.16512;
                public static final double topShooterKv = 0.11447;
                public static final double topShooterKa = 0.0035411;

                public static final double topShooterKf = 0.000198;

                public static final double shooterTolerance = 100.0;

                public static final double pivotKp = 0.1;
                public static final double pivotKi = 0.0015;
                public static final double pivotKd = 0.0;
                public static final double pivotKf = 0.45;
                public static final double pidPositionTolerance = 0.05;
                public static final double positionTolerance = 1;

                public static final double pivotParallelFloorOffset = 203.0478515625;

                public static final double pivotMaxVelocity = 1500.0;
                public static final double pivotMaxAcceleration = 1500.0;

                public static final int shooterBeamBrakeChannel = 0;
                public static final int shooterMotorCurrentLimit = 80;
                public static final int rollerMotorCurrentLimit = 80;

                public static final double armEncoderPositionConversionFactor = 360;
                public static final double armEncoderZeroOffset = 100;
                public static final boolean armEncoderInverted = true;
                public static final boolean shooterPivotMotorInverted = true;

                public static final double pivotMinPosition = 70;
                public static final double pivotMaxPosition = 240;
                public static final double pivotHomedPosition = 71;
                public static final double pivotIndexingPosition = 114;
                public static final double pivotAmpPosition = 183;
                public static final double pivotClimbPosition = 235;

                public static final double flywheelAmpRPM = 1000;

                public static final double rollerStoppedVelocity = 0.0;
                public static final double rollerIntakingVelocity = 0.3;
                public static final double rollerExhaustingVelocity = -0.1;
                public static final double rollerIdleVelocity = 0.1;

                public static final double shooterExhaustSpeed = 1000;
                public static final double shooterIdleSpeed = 5000;
                public static final double shooterStoppedSpeed = 0.0;

                public static final int forwardLimitSwitchID = 7;

                public static final double shootingRPM = 5000;
                /*
                 * public static final InterpolatingDoubleTreeMap shooterRPMMap = new
                 * InterpolatingDoubleTreeMap();
                 * 
                 * static {
                 * // Distance, RPM
                 * shooterRPMMap.put(1.018170626, 3000.0);
                 * shooterRPMMap.put(1.136725279, 3000.0);
                 * shooterRPMMap.put(1.547120287, 3000.0);
                 * shooterRPMMap.put(2.18457, 3000.0);
                 * shooterRPMMap.put(2.4435, 3000.0);
                 * shooterRPMMap.put(2.805406493, 3000.0);
                 * shooterRPMMap.put(3.031418358, 3000.0);
                 * shooterRPMMap.put(3.392893597, 3200.0);
                 * shooterRPMMap.put(3.812472063, 3400.0);
                 * shooterRPMMap.put(3.970896016, 3500.0);
                 * shooterRPMMap.put(4.311056677, 4500.0);
                 * 
                 * }
                 */
                public static final InterpolatingDoubleTreeMap shooterPivotMap = new InterpolatingDoubleTreeMap();

                static {
                        // Distance, Angle
                        shooterPivotMap.put(1.095979334274389, 150.0);
                        shooterPivotMap.put(1.282133452297385, 155.0);
                        shooterPivotMap.put(1.4342617709538774, 157.0);
                        shooterPivotMap.put(1.4342617709538774, 162.0);
                        shooterPivotMap.put(1.8687062887987802, 163.0);
                        shooterPivotMap.put(2.0448387211045267, 167.0);
                        shooterPivotMap.put(2.259941229448083, 169.0);
                        shooterPivotMap.put(2.3849374678265636, 171.0);
                        shooterPivotMap.put(2.6708614771998826, 173.0);
                        shooterPivotMap.put(2.8698417854532146, 175.0);
                        shooterPivotMap.put(3.0614379417757864, 176.0);
                        shooterPivotMap.put(3.21703461348205, 177.0);
                        shooterPivotMap.put(3.45334674644738, 178.0);
                        shooterPivotMap.put(3.805586983598703, 178.5);
                        shooterPivotMap.put(4.150999154490105, 179.0);
                        shooterPivotMap.put(4.532950393382253, 181.0);

                }

                public static final InterpolatingDoubleTreeMap shooterTimeMap = new InterpolatingDoubleTreeMap();

                static {
                        // Distance, Seconds
                        shooterTimeMap.put(1.095979334274389, 0.3);

                }
        }

        public static class ElevatorConstants {
                public static final int elevatorBoottomMotorID = 18;
                public static final int elevatorTopMotorID = 19;

                public static final int elevatorCANCoderID = 20;

                public static final double elevatorCANCoderOffset = -0.940000;

                public static final boolean elevatorTopMotorInverted = true;
                public static final boolean elevatorBottomMotorInverted = false;
                public static final int elevatorMotorCurrentLimit = 80;
                public static final double elevatorGearRatio = 24.6;

                public static final double elevatorKp = 1.79;
                public static final double elevatorKi = 0.015;
                public static final double elevatorKd = 0.0; // 0.00000000000000001;

                public static final double elevatorMinPosition = 0.24;
                public static final double elevatorMaxPosition = 44.5;
                public static final double elevatorTopPosition = 44;
                public static final double elevatorHomedPosition = 0.24;
                public static final double elevatorAmpPosition = 2;

                public static final double positionTolerance = 4;

                public static final SensorDirectionValue elevatorCANCoderSensorValue = SensorDirectionValue.CounterClockwise_Positive;
        }

        public static class IntakeConstants {
                public static final int intakeMotorID = 17;

                public static final double homedKp = 0.05;
                public static final double homedKi = 0.0;// 0.00004;
                public static final double homedKd = 0.0;

                public static final double extendedKp = 0.1;
                public static final double extendedKi = 0.0;
                public static final double extendedKd = 0.0;

                public static final double homedTolerance = 5;
                public static final double extendedTolerance = 2;

                public static final double extendedPIDMaxVelocity = 4500.0;
                public static final double extendedPIDMaxAcceleration = 4500.0;

                public static final double homedPIDMaxVelocity = 2500.0;
                public static final double homedPIDMaxAcceleration = 1500.0;

                public static final double minIntakePivotPosition = 137.400;
                public static final double maxIntakePivotPosition = 197.040;
                public static final double intakeHomedPosition = 140.00;
                public static final double intakeExtendedPosition = 195.000;

                public static final int intakePivotLeftMotorID = 14;
                public static final int intakePivotRightMotorID = 15;
                public static final int intakePushingMotor = 16;

                public static final double intakePivotEncoderPositionConversionFactor = 360;
                public static final double intakePivotEncoderZeroOffset = 0.0;

                public static final boolean intakePivotEncoderInverted = true;
                public static final boolean intakePivotLeftMotorInverted = false;
                public static final boolean intakePivotRightMotorInverted = true;

                public static final int intakeMotorsCurrentLimit = 30;

                public static final double intakeStoppedVelocity = 0.0;
                public static final double intakeIntakingVelocity = -0.7;
                public static final double intakeExhaustingVelocity = 0.5;
                public static final double intakeIdleVelocity = -0.1;

                public static final double pushingAcutatingPosition = 20;
                public static final double pushingHomedPosition = 1;

                public static final int pushingLimitSwitch = 9;
                public static final int intakeLimitSwitch = 8;
                public static final int intakeBeamBrakeChannel = 1;
        }

        public static class VisionConstants {
                public static final String aprilLimeNTName = "limelight-backcam";
                public static final String neuralLimeNTName = "limelight-intake";

                public static final int speakerBlueCenterTagID = 7;
                public static final int speakerRedCentgerTagID = 4;

                public static final int odometryPipelineNumber = 0;
                public static final int speakerPipelineNumber = 1;
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

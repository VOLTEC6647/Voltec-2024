/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.util;

import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Constants {

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kDriverControllerPort2 = 1;

        public static final CommandXboxController driverController1 = new CommandXboxController(
                OperatorConstants.kDriverControllerPort);
        public static final CommandXboxController driverController2 = new CommandXboxController(
                OperatorConstants.kDriverControllerPort2);
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

        public static final double maxSpeed = 5.4864;
        public static final double maxAcceleration = maxSpeed * 0.85;
        /** Radians per Second */
        public static final double maxAngularVelocity = 11.5;
        public static final double maxAngularAcceleration = 3.5;
        public static final AndromedaSwerveConfig andromedaSwerveConfig = new AndromedaSwerveConfig(0.1, trackWidth,
                wheelBase, swerveKinematics, maxSpeed, maxAcceleration, maxAngularVelocity, maxAngularAcceleration);
    }

}

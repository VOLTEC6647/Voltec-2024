/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperRobotState extends SubsystemBase{
    public void addOdometryObservations(double currentTimeSencods, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {

    }

    public void addVisionObservations() {

    }

    public static Pose2d getPose() {
        return null;
    }

    public static void resetPose(Pose2d pose) {

    }    
}

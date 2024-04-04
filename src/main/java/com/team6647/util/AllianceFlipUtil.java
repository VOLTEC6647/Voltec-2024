// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team6647.util;

import com.team6647.util.Constants.FieldConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {
    /**
     * Flips an x coordinate to the correct side of the field based on the current
     * alliance color.
     */
    public static double apply(double xCoordinate) {
        if (shouldFlip()) {
            return FieldConstants.fieldLength - xCoordinate;
        } else {
            return xCoordinate;
        }
    }

    /**
     * Flips a translation to the correct side of the field based on the current
     * alliance color.
     */
    public static Translation2d apply(Translation2d translation) {
        if (shouldFlip()) {
            return new Translation2d(apply(translation.getX()), translation.getY());
        } else {
            return translation;
        }
    }

    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d apply(Rotation2d rotation) {
        if (shouldFlip()) {
            return new Rotation2d(-rotation.getCos(), rotation.getSin());
        } else {
            return rotation;
        }
    }

    /**
     * Flips a pose to the correct side of the field based on the current alliance
     * color.
     */
    public static Pose2d apply(Pose2d pose) {
        if (shouldFlip()) {
            return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
        } else {
            return pose;
        }
    }

    /**
     * Forcefully flips a pose to the red side of the field regardless of the
     * current alliance color.
     * 
     * @param pose The pose to flip.
     * @return The flipped pose.
     */
    public static Pose2d forceApply(Pose2d pose) {
        return new Pose2d(forceApply(pose.getTranslation()), apply(pose.getRotation()));
    }

    /**
     * Forcefully flips a translation to the red side of the field regardless of the
     * current alliance color.
     * 
     * @param translation The translation to flip.
     * @return The flipped translation.
     */
    public static Translation2d forceApply(Translation2d translation) {
        return new Translation2d(forceApply(translation.getX()), translation.getY());
    }

    /**
     * Forcefully flips a rotation to the red side of the field regardless of the
     * current alliance color.
     * 
     * @param rotation The rotation to flip.
     * @return The flipped rotation.
     */
    public static Rotation2d forceApply(Rotation2d rotation) {
        return new Rotation2d(-rotation.getCos(), rotation.getSin());
    }

    /**
     * Forcefully flips an x coordinate to the red side of the field regardless of
     * the current alliance color.
     * 
     * @param xCoordinate The x coordinate to flip.
     * @return The flipped x coordinate.
     */
    public static double forceApply(double xCoordinate) {
        return FieldConstants.fieldLength - xCoordinate;
    }

    public static Translation3d apply(Translation3d translation3d) {
        if (shouldFlip()) {
            return new Translation3d(
                    apply(translation3d.getX()), translation3d.getY(), translation3d.getZ());
        } else {
            return translation3d;
        }
    }

    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
    }
}
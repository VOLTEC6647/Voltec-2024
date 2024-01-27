/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.andromedaSwerve.utils;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * References the IDs of the motors & angle offsets
 * 
 * @see AndromedaModuleConstants
 * 
 * Useful only for Team 6647's robot
 */
public class AndromedaMap {

        /* IDs are set clockwise starting from front right */
        public static final int steerID1 = 1;
        public static final int driveID1 = 2;
        public static final int absEncoder1 = 3;
        public static final Rotation2d mod1Angleoffset = Rotation2d.fromRotations(-0.018066); //0.553
        public static final AndromedModuleIDs mod1Const = new AndromedModuleIDs(steerID1, driveID1,
                        absEncoder1, mod1Angleoffset);

        public static final int steerID2 = 4;
        public static final int driveID2 = 5;
        public static final int absEncoder2 = 6;
        public static final Rotation2d mod2Angleoffset = Rotation2d.fromRotations(-0.315918); //3.540
        public static final AndromedModuleIDs mod2Const = new AndromedModuleIDs(steerID2, driveID2,
                        absEncoder2, mod2Angleoffset);

        public static final int steerID3 = 7;
        public static final int drivelID3 = 8;
        public static final int absEncoder3 = 9;
        public static final Rotation2d mod3Angleoffset = Rotation2d.fromRotations(-0.222656); //0.902;
        public static final AndromedModuleIDs mod3Const = new AndromedModuleIDs(steerID3, drivelID3,
                        absEncoder3, mod3Angleoffset);

        public static final int steerID4 = 10;
        public static final int driveID4 = 11;
        public static final int absEncoder4 = 12;
        public static final Rotation2d mod4Angleoffset = Rotation2d.fromRotations(-0.168945); //6.194
        public static final AndromedModuleIDs mod4Const = new AndromedModuleIDs(steerID4, driveID4,
                        absEncoder4, mod4Angleoffset);
}
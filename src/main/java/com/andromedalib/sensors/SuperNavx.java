/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Wrapper for the AHRS Navx class
 */
public class SuperNavx extends AHRS {

    private static SuperNavx instance;

    /**
     * Configures SuperNavx & resets it
     */
    private SuperNavx() {
        super();
        zeroHeading();
    }

    public static SuperNavx getInstance() {
        if (instance == null) {
            instance = new SuperNavx();
        }
        return instance;
    }

    /**
     * Resets the heading of the Navx
     */
    public void zeroHeading() {
        reset();
    }

    /**
     * Gets the heading of the Navx
     * 
     * @return Heading of the Navx in degrees
     */
    public double getHeading() {
        return -getRotation2d().getDegrees();
    }

    /**
     * Gets the turn rate of the Navx
     * 
     * @return Turn rate of the Navx in degrees per second
     */
    public double getTurnRate() {
        return getRate();
    }

    /**
     * Clamps the angle value
     * 
     * @return Angle value clamped around 360 degrees
     */
    public Rotation2d getClampedYaw() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(-getAngle(), 360));
/*         return Rotation2d.fromDegrees(-getYaw());
 */    }

    /**
     * Gets the current heading of the Navx
     * 
     * @return Current rotation as a negative
     *         {@link edu.wpi.first.math.geometry.Rotation2d}
     */
    public Rotation2d getRotation() {
        return getRotation2d();
    }
}

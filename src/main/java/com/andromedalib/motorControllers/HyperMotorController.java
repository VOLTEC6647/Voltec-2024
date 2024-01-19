/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.motorControllers;

import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;

/**
 * Interface to implement the same functions and
 * variables on all motor controllers
 */
public interface HyperMotorController { 

    /**
     * Sets the idle mode of the motor controller
     * 
     * @param idleMode Idle mode of the motor controller
     */
    public void setMode(GlobalIdleMode idleMode);

    /**
     * Gets the current limit of the motor controller
     * 
     * @return the current limit of the motor controller
     */
    public double getLimit();

    /**
     * Sets the current limit of the motor controller
     * 
     * @param currentLimit Current limit of the motor controller in amps
     */
    public void setLimit(int currentLimit);

    /*
     * Outputs Telemetry to SmartDashboard
     */
    public void outputTelemetry(String tabName);

    /**
     * Resets the encoder of the motor controller
     */
    public void resetEncoder();

}

/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.motorControllers;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

/**
 * Global class to manage the idle mode of all motors
 * This is to make it easier to manage motor states
 */
public class IdleManager {

    public enum GlobalIdleMode {
        Coast,
        Brake
    }

    /**
     * Converts from GlobalIdle mode to corresponding NeutralMode
     * 
     * @param idleMode The GlobalIdleMode to convert
     * 
     * @return The corresponding NeutralMode
     */
    static NeutralModeValue idleToNeutral(GlobalIdleMode idleMode) {
        return NeutralModeValue.valueOf(idleMode.ordinal());
    }

    /**
     * Converts from GlobalIdle mode to corresponding IdleMode
     * 
     * @param idleMode The GlobalIdleMode to convert
     * 
     * @return The corresponding IdleMode
     */
    static IdleMode neutralToIdle(GlobalIdleMode idleMode) {
        return IdleMode.values()[idleMode.ordinal()];
    }

}

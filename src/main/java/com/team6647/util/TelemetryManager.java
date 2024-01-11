/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 08 01 2023
 */

package com.team6647.util;

import com.andromedalib.robot.BaseTelemetryManager;

public class TelemetryManager extends BaseTelemetryManager{
    private static TelemetryManager instance;

    /**
     * Private Constructor
     */
    private TelemetryManager() {

    }

    public static TelemetryManager getInstance() {
        if (instance == null) {
            instance = new TelemetryManager();
        }
        return instance;
    }
}

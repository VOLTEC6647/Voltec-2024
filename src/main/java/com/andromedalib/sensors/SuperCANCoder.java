/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.sensors;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

public class SuperCANCoder extends CANcoder {

    /**
     * SuperCANCoder constructor
     * 
     * @param deviceID      CANCoder DeviceID
     * @param configuration Configuration
     */
    public SuperCANCoder(int deviceID, CANcoderConfiguration configuration) {
        super(deviceID);

        getConfigurator().apply(new CANcoderConfiguration());
        getConfigurator().apply(configuration);
    }

    /**
     * SuperCANCoder constructor to a defined canbus
     * 
     * @param deviceID      CANCoder DeviceID
     * @param configuration Configuration
     * @param canbus        Device canbus
     */
    public SuperCANCoder(int deviceID, CANcoderConfiguration configuration, String canbus) {
        super(deviceID, canbus);

        getConfigurator().apply(new CANcoderConfiguration());
        getConfigurator().apply(configuration);
    }

}

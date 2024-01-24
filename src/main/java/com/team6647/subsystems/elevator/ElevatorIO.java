/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 01 2024
 */

package com.team6647.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.team6647.util.Constants.ElevatorConstants;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorPosition = ElevatorConstants.elevatorMinPosition;
        public double elevatorAbsoluteEncoderPosition = 0.0;
        public double elevatorAppliedVolts = 0.0;

        public boolean topBeambrake = false;
        public boolean bottomBeamBrake = false;
    }

    /* Updates the set of loggable inputs s */
    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void setElevatorPosition(double position) {
    }
}

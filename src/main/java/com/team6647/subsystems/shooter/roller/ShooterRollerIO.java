/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 16 02 2024
 */

package com.team6647.subsystems.shooter.roller;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterRollerIO {

    @AutoLog
    public static class ShooterRollerIOInputs {
        public double rollerVelocity = 0.0;

        public double rollerCurrent = 0.0;
    }

    public default void updateInputs(ShooterRollerIOInputs inputs) {
    }

    public default void setRollerVelocity(double percentOutput) {
    }
}

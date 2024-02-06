/**
 * Written by Juan Pablo Gutiérrez  
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {

    @AutoLog
    public static class ShooterPivotIOInputs {
        public double shooterAbsoluteEncoderPosition = 0.0;
    }

    public void updateInputs(ShooterPivotIOInputs inputs);

    public void setShooterVoltage(double volts);

}

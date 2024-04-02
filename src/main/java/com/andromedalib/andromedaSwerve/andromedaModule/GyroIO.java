package com.andromedalib.andromedaSwerve.andromedaModule;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityDegrees = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {
    }

    public default void setGyroAngle(Rotation2d angle) {
    }
}

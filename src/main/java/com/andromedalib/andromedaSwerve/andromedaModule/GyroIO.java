package com.andromedalib.andromedaSwerve.andromedaModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public MutableMeasure<Angle> yawPosition = MutableMeasure.zero(Rotations);
        public MutableMeasure<Velocity<Angle>> yawVelocity = MutableMeasure.zero(RadiansPerSecond);
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {
    }

    public default void setGyroAngle(Rotation2d angle) {
    }
}

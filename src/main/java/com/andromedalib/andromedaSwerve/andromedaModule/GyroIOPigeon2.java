/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 22 03 2024
 */

package com.andromedalib.andromedaSwerve.andromedaModule;

import java.util.Queue;

import com.andromedalib.andromedaSwerve.utils.PhoenixOdometryThread;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;
    private final StatusSignal<Double> yaw;
    private final StatusSignal<Double> yawVelocity;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    /**
     * Constructs a new GyroIOPigeon2
     * 
     * @param gyroID Gyro CAN ID
     */
    public GyroIOPigeon2(int id, String canbus) {
        pigeon = new Pigeon2(id, canbus);
        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(100.0);
        yawVelocity.setUpdateFrequency(100.0);
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon, pigeon.getYaw());

    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityDegrees = yawVelocity.getValueAsDouble();

        inputs.odometryYawTimestamps =
            yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
            yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();

    }

    @Override
    public void setGyroAngle(Rotation2d angle) {
        pigeon.getConfigurator().setYaw(angle.getDegrees());
    }
}

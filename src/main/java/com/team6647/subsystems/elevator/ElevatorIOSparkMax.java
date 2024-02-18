/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 01 2024
 */

package com.team6647.subsystems.elevator;

import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.andromedalib.motorControllers.SuperSparkMax;
import com.team6647.util.Constants.ElevatorConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
    private static SuperSparkMax elevatorTopMotor = new SuperSparkMax(
            ElevatorConstants.elevatorTopMotorID,
            GlobalIdleMode.Brake,
            ElevatorConstants.elevatorTopMotorInverted,
            ElevatorConstants.elevatorMotorCurrentLimit);

    private static SuperSparkMax elevatorBottomMotor = new SuperSparkMax(
            ElevatorConstants.elevatorBoottomMotorID,
            GlobalIdleMode.Brake,
            ElevatorConstants.elevatorTopMotorInverted,
            ElevatorConstants.elevatorMotorCurrentLimit);

    private static CANcoder elevatorCANCoder = new CANcoder(ElevatorConstants.elevatorCANCoderID);

    public ElevatorIOSparkMax() {
        configCANCoder();

        elevatorTopMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ElevatorConstants.elevatorMaxPosition);
        elevatorTopMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ElevatorConstants.elevatorMinPosition);

        elevatorBottomMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ElevatorConstants.elevatorMaxPosition);
        elevatorBottomMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ElevatorConstants.elevatorMinPosition);

        setElevatorPosition();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorPosition = elevatorTopMotor.getPosition();
        inputs.elevatorAbsoluteEncoderPosition = elevatorCANCoder.getAbsolutePosition().getValueAsDouble();

        inputs.elevatorAppliedVolts = elevatorTopMotor.getBusVoltage();
    }

    @Override
    public void setElevatorVoltage(double volts) {
        elevatorTopMotor.setVoltage(volts);
        elevatorBottomMotor.setVoltage(volts);
    }

    public void configCANCoder() {
        elevatorCANCoder.getConfigurator().apply(new CANcoderConfiguration());
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = ElevatorConstants.elevatorCANCoderSensorValue;
        config.MagnetSensor.MagnetOffset = ElevatorConstants.elevatorCANCoderOffset;
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        elevatorCANCoder.getConfigurator().apply(config);
    }

    public void setElevatorPosition() {
        elevatorTopMotor.setPosition(0);

        double absPos = elevatorCANCoder.getAbsolutePosition().getValueAsDouble();

        elevatorTopMotor.setPosition(absPos);
    }
}

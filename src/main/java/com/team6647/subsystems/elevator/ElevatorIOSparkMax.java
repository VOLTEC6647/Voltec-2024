/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 01 2024
 */

package com.team6647.subsystems.elevator;

import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.andromedalib.motorControllers.SuperSparkMax;
import com.team6647.util.Constants.ElevatorConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public class ElevatorIOSparkMax implements ElevatorIO {
    private static SuperSparkMax elevatorTopMotor = new SuperSparkMax(
            ElevatorConstants.elevatorTopMotorID,
            GlobalIdleMode.brake,
            ElevatorConstants.elevatorTopMotorInverted,
            ElevatorConstants.elevatorMotorCurrentLimit);

    private static SuperSparkMax elevatorBottomMotor = new SuperSparkMax(
            ElevatorConstants.elevatorBoottomMotorID,
            GlobalIdleMode.brake,
            ElevatorConstants.elevatorTopMotorInverted,
            ElevatorConstants.elevatorMotorCurrentLimit);

    private static CANcoder elevatorCANCoder = new CANcoder(ElevatorConstants.elevatorCANCoderID);

    private static Rotation2d elevatorOffset = new Rotation2d(ElevatorConstants.elevatorCANCoderOffset);

    public ElevatorIOSparkMax() {

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorPosition = elevatorTopMotor.getPosition();
        inputs.elevatorAbsoluteEncoderPosition = elevatorCANCoder.getAbsolutePosition().getValueAsDouble()
                - elevatorOffset.getRotations();

        inputs.elevatorAppliedVolts = elevatorTopMotor.getBusVoltage();
    }

    @Override
    public void setElevatorVoltage(double volts) {
        elevatorTopMotor.setVoltage(volts);
        elevatorBottomMotor.setVoltage(volts);
    }
}

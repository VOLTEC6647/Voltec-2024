/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 24 02 2024
 */

package com.team6647.subsystems.flywheel;

import com.andromedalib.motorControllers.SuperTalonFX;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterIOKraken implements ShooterIO {

    private static SuperTalonFX topFlywheelMotor = new SuperTalonFX(
            ShooterConstants.flywheelTopMotorID,
            GlobalIdleMode.Coast,
            true);

    private static SuperTalonFX botttomFlywheelMotor = new SuperTalonFX(
            ShooterConstants.flywheelBottomMotorID,
            GlobalIdleMode.Coast,
            true);

    private static DigitalInput shooterBeamBrake = new DigitalInput(ShooterConstants.shooterBeamBrakeChannel);

    private VelocityVoltage topFlywheelVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    private VelocityVoltage bottomFlywheelVelocityVoltage = new VelocityVoltage(0).withSlot(0);

    private SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0, 0);
    private SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0, 0);

    public ShooterIOKraken() {
        topFlywheelMotor.getConfigurator().apply(new TalonFXConfiguration());
        botttomFlywheelMotor.getConfigurator().apply(new TalonFXConfiguration());

        setBottomPIDF(ShooterConstants.bottomShooterKp, ShooterConstants.bottomShooterKi, ShooterConstants.bottomShooterKd,
                ShooterConstants.bottomShooterKs, ShooterConstants.bottomShooterKv, ShooterConstants.bottomShooterKa);

        setTopPIDF(ShooterConstants.topShooterKp, ShooterConstants.topShooterKi, ShooterConstants.topShooterKd,
                ShooterConstants.topShooterKs, ShooterConstants.topShooterKv, ShooterConstants.topShooterKa);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.bottomMotorVelocity = botttomFlywheelMotor.getVelocity().getValue() * 60;
        inputs.topMotorVelocity = topFlywheelMotor.getVelocity().getValue() * 60;

        inputs.beamBrake = shooterBeamBrake.get();
    }

    @Override
    public void setShooterVelocity(double velocity) {
        topFlywheelMotor.setControl(
                topFlywheelVelocityVoltage.withVelocity(velocity).withFeedForward(topFeedforward.calculate(velocity)));
        botttomFlywheelMotor
                .setControl(bottomFlywheelVelocityVoltage.withVelocity(velocity)
                        .withFeedForward(bottomFeedforward.calculate(velocity)));
    }

    @Override
    public void setTopPIDF(double p, double i, double d, double s, double v, double a) {
        TalonFXConfiguration topMotorConfig = new TalonFXConfiguration();

        topMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        topMotorConfig.Slot0.kP = p;
        topMotorConfig.Slot0.kI = i;
        topMotorConfig.Slot0.kD = d;

        topFeedforward = new SimpleMotorFeedforward(s, v, a);
        bottomFeedforward = new SimpleMotorFeedforward(s, v, a);

        topFlywheelMotor.getConfigurator().apply(topMotorConfig);
    }

    @Override
    public void setBottomPIDF(double p, double i, double d, double s, double v, double a) {
        TalonFXConfiguration bottomMotorConfig = new TalonFXConfiguration();

        bottomMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        bottomMotorConfig.Slot0.kP = p;
        bottomMotorConfig.Slot0.kI = i;
        bottomMotorConfig.Slot0.kD = d;

        bottomFeedforward = new SimpleMotorFeedforward(s, v, a);

        botttomFlywheelMotor.getConfigurator().apply(bottomMotorConfig);
    }
}

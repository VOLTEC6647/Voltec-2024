/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.motorControllers;

import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * Wrapper for the CANSparkMax class
 * That implements the HyperMotorController interface
 */
public class SuperSparkMax extends CANSparkMax implements HyperMotorController {

    private double currentLimit = 1;

    RelativeEncoder encoder = getEncoder();

    REVLibError error;

    /**
     * Configures SuperSparkMax motor controller
     * 
     * @param id           ID of the motor controller
     * @param type         Type of the motor controller
     * @param idleMode     Idle mode of the motor controller
     * @param inverted     Inverted state of the motor controller
     * @param currentLimit Current limit of the motor controller in amps
     */
    public SuperSparkMax(int motorID, MotorType type, GlobalIdleMode idleMode, boolean isInverted, int currentLimit) {
        super(motorID, type);
        restoreFactoryDefaults();
        setMode(idleMode);
        setInverted(isInverted);
        setSmartCurrentLimit(currentLimit);
        burnFlash();
    }

    /**
     * Configures SuperSparkMax motor controller
     * 
     * @param id       ID of the motor controller
     * @param type     Type of the motor controller
     * @param idleMode Idle mode of the motor controller
     * @param inverted Inverted state of the motor controller
     */
    public SuperSparkMax(int motorID, MotorType type, GlobalIdleMode idleMode, boolean isInverted) {
        super(motorID, type);
        restoreFactoryDefaults();
        setMode(idleMode);
        setInverted(isInverted);
        burnFlash();
    }

    /**
     * Configures SuperSparkMax motor controller with the {@link IdleMode} set to
     * Coast, the {@link MotorType} set to brushless
     *
     * @param motorID      ID of the motor controller
     * @param inverted     Inverted state of the motor controller
     * @param currentLimit Current limit of the motor controller in amps
     */
    public SuperSparkMax(int motorID, GlobalIdleMode mode, boolean isInverted, int currentLimit) {
        super(motorID, MotorType.kBrushless);
        restoreFactoryDefaults();
        encoder.setPosition(0);
        setMode(mode);
        setInverted(isInverted);
        setSmartCurrentLimit(currentLimit);
        burnFlash();
    }

    /**
     * Configures SuperSparkMax motor controller with the {@link IdleMode} set to
     * Coast, the {@link MotorType} set to brushless. It also configures the
     * absolute encoder
     *
     * @param motorID                          ID of the motor controller
     * @param inverted                         Inverted state of the motor
     *                                         controller
     * @param currentLimit                     Current limit of the motor controller
     *                                         in amps
     * @param absolutePositionConversionFactor Absolute Position Conversion Factor
     * @param zeroOffset                       Zero Offset
     */
    public SuperSparkMax(int motorID, GlobalIdleMode mode, boolean isInverted, int currentLimit,
            double absolutePositionConversionFactor, double zeroOfsset, boolean encoderInverted) {
        super(motorID, MotorType.kBrushless);
        restoreFactoryDefaults();
        encoder.setPosition(0);
        setMode(mode);
        setInverted(isInverted);
        setSmartCurrentLimit(currentLimit);
        getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setPositionConversionFactor(absolutePositionConversionFactor);
        getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(zeroOfsset);
        getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setInverted(encoderInverted);
        burnFlash();
    }

    /**
     * Configures SuperSparkMax motor controller with the {@link IdleMode} set to
     * Coast, the {@link MotorType} set to brushless
     *
     * @param motorID      ID of the motor controller
     * @param inverted     Inverted state of the motor controller
     * @param currentLimit Current limit of the motor controller in amps
     */
    public SuperSparkMax(int motorID, boolean isInverted, int currentLimit) {
        super(motorID, MotorType.kBrushless);
        restoreFactoryDefaults();
        setMode(GlobalIdleMode.Coast);
        setInverted(isInverted);
        setSmartCurrentLimit(currentLimit);
        burnFlash();
    }

    /**
     * Configures SuperSparkMax motor controller without a current limit. The
     * {@link IdleMode} set to
     * Coast, the {@link MotorType} set to brushless
     *
     * @param motorID  ID of the motor controller
     * @param inverted Inverted state of the motor controller
     */
    public SuperSparkMax(int motorID, boolean isInverted) {
        super(motorID, MotorType.kBrushless);
        restoreFactoryDefaults();
        setMode(GlobalIdleMode.Coast);
        setInverted(isInverted);
        burnFlash();
    }

    @Override
    public void outputTelemetry(String tabName) {
        Shuffleboard.getTab(tabName).add("NEO Motor " + getDeviceId() + "Voltage:", getBusVoltage());
        Shuffleboard.getTab(tabName).add("NEO Motor " + getDeviceId() + "Temperature", getMotorTemperature());
    }

    /**
     * Sets the idle mode of the motor controller
     * 
     * @param idleMode Idle mode of the motor controller
     */
    @Override
    public void setMode(GlobalIdleMode idleMode) {
        setIdleMode(IdleManager.neutralToIdle(idleMode));
    }

    /**
     * Sets the current limit of the motor controller
     * 
     * @param currentLimit Current limit of the motor controller in amps
     */
    @Override
    public void setLimit(int currentLimit) {
        this.currentLimit = currentLimit;
    }

    /**
     * Gets the current limit of the motor controller
     * 
     * @return the current limit of the motor controller
     */
    @Override
    public double getLimit() {
        return currentLimit;
    }

    /**
     * Sets the position ƒconversion factor for the encoder
     * 
     * @param conversionFactor Conversion factor for the encoder
     */
    public void setPositionConversionFactor(double conversionFactor) {
        encoder.setPositionConversionFactor(conversionFactor);
    }

    /**
     * Sets the velocity conversion factor for the encoder
     * 
     * @param conversionFactor Conversion factor for the encoder
     */
    public void setVelocityConversionFactor(double conversionFactor) {
        encoder.setVelocityConversionFactor(conversionFactor);
    }

    /**
     * Gets the position of the encoder
     * 
     * @return the position of the encoder
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * Swts the encoder to a defined position
     * 
     * @param position New position
     */
    public void setPosition(double position) {
        encoder.setPosition(position);
    }

    /**
     * Gets the velocity of the encoder
     * 
     * @return the velocity of the encoder
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Sets the encoder position to 0
     * 
     */
    @Override
    public void resetEncoder() {
        encoder.setPosition(0);
    }
}
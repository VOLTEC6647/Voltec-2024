/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.motorControllers;

import com.andromedalib.math.Conversions;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * Wrapper for the WPI_TalonFX class
 * That implements the HyperMotorController interface
 */
public class SuperTalonFX extends TalonFX implements HyperMotorController {

    private double currentLimit = 1;

    /**
     * Configures SuperTalonFX motor controller
     * 
     * @param motorID       ID of the motor controller
     * @param idleMode      Idle mode of the motor controller
     * @param inverted      Inverted state of the motor controller
     * @param configuration Stator current limit configuration
     */
    public SuperTalonFX(int motorID, GlobalIdleMode idleMode, Boolean isInverted,
            TalonFXConfiguration configuration) {
        super(motorID);
        getConfigurator().apply(new TalonFXConfiguration());
        setMode(idleMode);
        setInverted(isInverted);
        getConfigurator().apply(configuration);
    }

    /**
     * Configures SuperTalonFX motor controller to a defined CAN bus
     * 
     * @param motorID       ID of the motor controller
     * @param idleMode      Idle mode of the motor controller
     * @param inverted      Inverted state of the motor controller
     * @param configuration Stator current limit configuration
     * @param canbus        Device Canbus
     */
    public SuperTalonFX(int motorID, GlobalIdleMode idleMode, Boolean isInverted,
            TalonFXConfiguration configuration, String canbus) {
        super(motorID, canbus);
        getConfigurator().apply(new TalonFXConfiguration());
        setMode(idleMode);
        setInverted(isInverted);
        getConfigurator().apply(configuration);
    }

    /**
     * Configures SuperTalonFX motor controller
     * 
     * @param motorID    ID of the motor controller
     * @param idleMode   Idle mode of the motor controller
     * @param isInverted Inverted state of the motor controller
     */
    public SuperTalonFX(int motorID, GlobalIdleMode idleMode, Boolean isInverted) {
        super(motorID);
        getConfigurator().apply(new TalonFXConfiguration());
        setMode(idleMode);
        setInverted(isInverted);
    }

    /**
     * Configures SuperTalonFX motor controller to a defined CAN bus
     * 
     * @param motorID    ID of the motor controller
     * @param idleMode   Idle mode of the motor controller
     * @param isInverted Inverted state of the motor controller
     * @param canbus     Device canbus
     */
    public SuperTalonFX(int motorID, GlobalIdleMode idleMode, Boolean isInverted, String canbus) {
        super(motorID, canbus);
        getConfigurator().apply(new TalonFXConfiguration());
        setMode(idleMode);
        setInverted(isInverted);
    }

    /**
     * Configures a SuperTalonFX with the {@link NeutralMode} as Coast
     * 
     * @param motorID
     * @param isInverted
     * @param configuration
     */
    public SuperTalonFX(int motorID, boolean isInverted, TalonFXConfiguration configuration) {
        super(motorID);
        getConfigurator().apply(new TalonFXConfiguration());
        setMode(GlobalIdleMode.Coast);
        setInverted(isInverted);
        getConfigurator().apply(configuration);

    }

    /**
     * Configures a SuperTalonFX with the {@link NeutralMode} as Coast to a defined
     * canbus
     * 
     * @param motorID
     * @param isInverted
     * @param configuration
     * @param canbus
     */
    public SuperTalonFX(int motorID, boolean isInverted, TalonFXConfiguration configuration, String canbus) {
        super(motorID, canbus);
        getConfigurator().apply(new TalonFXConfiguration());
        setMode(GlobalIdleMode.Coast);
        setInverted(isInverted);
        getConfigurator().apply(configuration);

    }

    /**
     * Configures a SuperTalonFX with the {@link NeutralMode} as Coast
     * 
     * @param motorID
     * @param isInverted
     * @param configuration
     */
    public SuperTalonFX(int motorID, TalonFXConfiguration configuration) {
        super(motorID);
        getConfigurator().apply(new TalonFXConfiguration());
        setMode(GlobalIdleMode.Coast);
        getConfigurator().apply(configuration);

    }

    /**
     * Configures a SuperTalonFX with the {@link NeutralMode} as Coast to a defined
     * canbus
     * 
     * @param motorID
     * @param isInverted
     * @param configuration
     */
    public SuperTalonFX(int motorID, TalonFXConfiguration configuration, String canbus) {
        super(motorID, canbus);
        getConfigurator().apply(new TalonFXConfiguration());
        setMode(GlobalIdleMode.Coast);
        getConfigurator().apply(configuration);

    }

    /**
     * Configures a SuperTalonFX
     * @param driveMotorID ID of the motor controller
     * @param mode Idle mode of the motor controller
     * @param driveMotorConfiguration Motor configuration
     * @param swerveCANBus Device canbus
     */
    public SuperTalonFX(int driveMotorID, GlobalIdleMode mode, TalonFXConfiguration driveMotorConfiguration,
            String swerveCANBus) {
        super(driveMotorID, swerveCANBus);
        getConfigurator().apply(new TalonFXConfiguration());
        setMode(mode);
        getConfigurator().apply(driveMotorConfiguration);
    }

    @Override
    public void outputTelemetry(String tabName) {
        Shuffleboard.getTab(tabName).add("TalonFX Motor " + getDeviceID() + "Voltage:", getSupplyVoltage());
        Shuffleboard.getTab(tabName).add("TalonFX Motor " + getDeviceID() + "Temperature", getDeviceTemp());
    }

    /**
     * Sets the idle mode of the motor controller
     * 
     * @param idleMode Idle mode of the motor controller
     */
    @Override
    public void setMode(GlobalIdleMode idleMode) {
        MotorOutputConfigs configs = new MotorOutputConfigs();
        configs.NeutralMode = IdleManager.idleToNeutral(idleMode);
        getConfigurator().apply(configs);
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
     * Gets the position of the motor controller adjusted to a circumference (Useful
     * for tank drives)
     * 
     * @param circumference Circumference of the wheel in meters
     * @param gearRatio     Gear ratio of the motor controller
     * 
     * @return the position of the motor in meters
     */
    public double getPosition(double circumference, double gearRatio) {
        return Conversions.falconToMeters(getPosition().getValue(), circumference, gearRatio);
    }

    /**
     * Gets the motor angle adjusted to the gear ratio
     * 
     * @param gearRatio Gear ratio of the mechanism
     * 
     * @return the angle of the motor in degrees
     */
    public double getAngle(double gearRatio) {
        return Conversions.falconToDegrees(getPosition().getValue(), gearRatio);
    }

    /**
     * Gets the velocity of the motor controller
     * 
     * @param circumference Circumference of the wheel in meters
     * @param gearRatio     Gear ratio of the motor controller
     * 
     * @return the velocity of the motor in meters per second
     */
    public double getVelocity(double circumference, double gearRatio) {
        return Conversions.falconToMPS(getPosition().getValue(), circumference, gearRatio);
    }

    /**
     * Resets the encoder of the motor controller
     */
    @Override
    public void resetEncoder() {
        setPosition(0);
    }

}
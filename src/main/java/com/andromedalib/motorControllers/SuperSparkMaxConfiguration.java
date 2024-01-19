package com.andromedalib.motorControllers;

public class SuperSparkMaxConfiguration {
    public final double positionConversionFactor;

    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;

    public final boolean inverted;

    public final double continousCurrentLimit;

    public SuperSparkMaxConfiguration(double positionConversionFactor, double kP,
            double kI, double kD, double kF, boolean inverted, double continousCurrentLimit) {
        this.positionConversionFactor = positionConversionFactor;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.inverted = inverted;
        this.continousCurrentLimit = continousCurrentLimit;
    }
}

/**
 * Written by Juan Pablo Gutierrez
 */

package com.andromedalib.sensors;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * A class that represents revs through bore encoder in the form of an absolute
 * encoder. Connect the PWM cable to the Rio's DIO port 
 */
public class RevAbsoluteEncoder {
    private DutyCycleEncoder encoder;

    /**
     * Creates a new Absolute Encoder
     * 
     * @param dioChannel Connected DIO Channel
     */
    public RevAbsoluteEncoder(int dioChannel) {
        encoder = new DutyCycleEncoder(dioChannel);
        encoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);
        encoder.setDistancePerRotation(360);
    }

    /**
     * Gets the absolute position in degrees
     * 
     * @return Absolute position
     */
    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition() * 360;
    }
}

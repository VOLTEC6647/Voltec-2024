/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.leds;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Wrapper for the Blinkin LED controller
 */
public class Blinkin extends SubsystemBase {

  static Blinkin instance;

  private static Spark blinkin;

  /**
   * Setup the Blinkin LED controller
   */
  private Blinkin(int pwmPort) {
    blinkin = new Spark(pwmPort);
    solidBlack();
  }

  public static Blinkin getInstance(int pwmPort) {
    if (instance == null) {
      instance = new Blinkin(pwmPort);
    }
    return instance;
  }

  @Override
  public void periodic() {
  }

  /**
   * Sets the LED to a decired pattern
   * 
   * @param pwmSignal The PWM signal to set the LED to
   */
  public void set(double pwmSignal) {
    blinkin.set(pwmSignal);
  }

  /**
   * Sets the LED to fire
   */
  public void fire() {
    blinkin.set(-0.59);
  }

  /*
   * Sets the LED to solid black
   */
  public void solidBlack() {
    blinkin.set(-0.73);

  }

  /*
   * Sets the LED to breathe blue
   */
  public void breathBlue() {
    blinkin.set(-0.15);

  }

  /*
   * Sets the LED to dark green
   */
  public void darkGreen() {
    blinkin.set(0.75);
  }

  /*
   * Sets the LED to hearbeat fast
   */
  public void Heartbeatfast() {
    blinkin.set(0.07);
  }

  /*
   * Sets the LED to light chase blue
   */
  public void LightChaseBlue() {
    blinkin.set(-0.29);
  }

  /*
   * Sets the LED to larson scanner red
   */
  public void larsonScannerRed() {
    blinkin.set(-0.35);
  }

  /*
   * Sets the LED to strobe gold
   */
  public void strobeGold() {
    blinkin.set(-0.07);
  }

  /*
   * Sets the LED to the alliance color
   */
  public void allianceColor() {
    boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
    if (isRed == true) {
      blinkin.set(-0.39);
      System.out.println("Color Waves, Lava Palette");
    } else {
      blinkin.set(-0.23);
      System.out.println("Heartbeat, Blue");
    }
  }
}

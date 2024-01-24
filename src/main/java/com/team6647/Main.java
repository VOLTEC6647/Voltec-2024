/**
 * Teamm 6647
 * 
 * Voltec 2024
 * 
 * Written by 
 * Juan Pablo Gutiérrez
 * Mauricio Villarreal 
 * 
 * Started: 08 01 2024
 */
package com.team6647;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}

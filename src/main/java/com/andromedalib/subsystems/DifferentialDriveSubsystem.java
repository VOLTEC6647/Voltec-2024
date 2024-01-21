/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DifferentialDriveSubsystem extends SubsystemBase {
  public static MotorControllerGroup leftMotorController;
  public static MotorControllerGroup rightMotorController;

  private double leftSpeed = 0.0;
  private double rightSpeed = 0.0;

  private DifferentialDrive drive;

  private boolean driveInverted = false;

  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  /**
   * Initializes the DifferentialDriveSubsystem with one side inverted
   * 
   * @param leftMotors   Array of left side motors
   * @param rightMotors  Array of right side motors
   * @param invertedSide Side which will be inverted. Left/Right. Case sensitive
   */
  public DifferentialDriveSubsystem(MotorController[] leftMotors, MotorController[] rightMotors, String invertedSide) {
    leftMotorController = new MotorControllerGroup(leftMotors);
    rightMotorController = new MotorControllerGroup(rightMotors);
    switch (invertedSide) {
      case "Left":
        leftMotorController.setInverted(true);
        rightMotorController.setInverted(false);
        break;
      case "Right":
        leftMotorController.setInverted(false);
        rightMotorController.setInverted(true);
        break;
      default:
        DriverStation.reportError("DifferentialDrive, inverted command not found", false);
        break;
    }
    drive = new DifferentialDrive(leftMotorController, rightMotorController);

    powerOnCompressor();
  }

  /**
   * Publishes data and information to Dashboard
   */
  @Override
  public void periodic() {
    outputTelemetry();
  }

  /**
   * Gets the DifferentialDrive
   * 
   * @return The DifferentialDrive instance
   */
  public DifferentialDrive getDrive() {
    return drive;
  }

  /**
   * Gets the Left side speed
   * 
   * @return Left side speed
   */
  public double getLeftSpeed() {
    return leftSpeed;
  }

  /**
   * Gets the right side speed
   * 
   * @return Right side speed
   */
  public double getRightSpeed() {
    return rightSpeed;
  }

  /**
   * Publishes data and information to SmartDashboard. Override this method to
   * include custom information
   */
  public void outputTelemetry() {
  }

  /**
   * Tank drive
   * 
   * @param leftSpeed  Left side speed
   * @param rightSpeed Right side speed
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    drive.tankDrive(this.leftSpeed, this.rightSpeed);
  }

  /**
   * Arcade drive
   * 
   * @param linearSpeed X speed
   * @param rotSpeed    Rotational value
   */
  public void arcadeDrive(double linearSpeed, double rotSpeed) {
    this.leftSpeed = linearSpeed;
    this.rightSpeed = rotSpeed;
    drive.arcadeDrive(leftSpeed, rightSpeed);
  }

  /**
   * Curvature Drive. Allows turn in place
   * 
   * @param linearSpeed X speed
   * @param rotSpeed    Rotational value
   */
  public void curvatureDrive(double linearSpeed, double rotSpeed) {
    this.leftSpeed = linearSpeed;
    this.rightSpeed = rotSpeed;
    drive.curvatureDrive(leftSpeed, rightSpeed, true);
  }

  /**
   * Returns whether the drive is inverted or not. Useful for the tank drive
   * inverted commands
   * 
   * @return Current inverted state
   */
  public boolean isInverted() {
    return driveInverted;
  }

  /**
   * Toggles inverted state
   */
  public void toggleInverted() {
    driveInverted = !driveInverted;
  }

  /**
   * Powers on the Compressor
   */
  public void powerOnCompressor() {
    compressor.enableDigital();
  }

  /**
   * Powers off the Compressor
   */
  public void powerOffCompressor() {
    compressor.disable();
  }

}
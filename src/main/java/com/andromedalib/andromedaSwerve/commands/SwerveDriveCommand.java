/**
 * Written by Juan Pablo Gutiérrez
 */
package com.andromedalib.andromedaSwerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.andromedalib.math.Functions;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveCommand extends Command {

  AndromedaSwerve swerve;
  DoubleSupplier translationY, translationX, rotation;
  BooleanSupplier fieldOriented;

  SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  /** Creates a new SwerveDrive. */
  public SwerveDriveCommand(AndromedaSwerve andromedaSwerve, DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier rotation, BooleanSupplier fieldOrientedControl) {
    this.swerve = andromedaSwerve;
    this.translationY = translationY;
    this.translationX = translationX;
    this.rotation = rotation;
    this.fieldOriented = fieldOrientedControl;

    xLimiter = new SlewRateLimiter(andromedaSwerve.andromedaProfile.maxAcceleration);
    yLimiter = new SlewRateLimiter(andromedaSwerve.andromedaProfile.maxAcceleration);
    turningLimiter = new SlewRateLimiter(andromedaSwerve.andromedaProfile.maxAngularAcceleration);

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double translationYVal = Functions.handleDeadband(translationY.getAsDouble(), swerve.andromedaProfile.deadband);
    double translationXVal = Functions.handleDeadband(translationX.getAsDouble(), swerve.andromedaProfile.deadband);
    double rotationVal = Functions.handleDeadband(rotation.getAsDouble(), swerve.andromedaProfile.deadband);

    double ySpeed = yLimiter.calculate(translationYVal);
    double xSpeed = xLimiter.calculate(translationXVal);
    double rotationSpeed = turningLimiter.calculate(rotationVal);

    swerve.drive(
        new Translation2d(ySpeed, xSpeed).times(swerve.andromedaProfile.maxSpeed),
        rotationSpeed * swerve.andromedaProfile.maxAngularVelocity,
        !fieldOriented.getAsBoolean());
  }

}

/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.subsystems;

import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team6647.util.Constants.DriveConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSubsystem extends SubsystemBase {
  private static AutoSubsystem instance;

  private final AndromedaSwerve andromedaSwerve;

  /** Creates a new AutoSubsystem. */
  private AutoSubsystem(AndromedaSwerve swerve) {
    this.andromedaSwerve = swerve;

    AutoBuilder.configureHolonomic(
        swerve::getPose,
        swerve::resetPose,
        swerve::getRelativeChassisSpeeds,
        swerve::drive,
        DriveConstants.holonomicPathConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, andromedaSwerve);
  }

  public static AutoSubsystem getInstance(AndromedaSwerve swerve) {
    if (instance == null) {
      instance = new AutoSubsystem(swerve);
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

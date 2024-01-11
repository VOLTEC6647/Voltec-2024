/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 08 01 2023
 */
package com.team6647;

import com.andromedalib.andromedaSwerve.andromedaModule.FalconAndromedaModule;
import com.andromedalib.andromedaSwerve.commands.SwerveDriveCommand;
import com.andromedalib.andromedaSwerve.config.AndromedaModuleConfig;
import com.andromedalib.andromedaSwerve.config.AndromedaModuleConfig.AndromedaProfiles;
import com.andromedalib.andromedaSwerve.subsystems.AndromedaSwerve;
import com.andromedalib.andromedaSwerve.utils.AndromedaMap;
import com.andromedalib.robot.SuperRobotContainer;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.Constants.OperatorConstants;

public class RobotContainer extends SuperRobotContainer {
  private static RobotContainer instance;

  private AndromedaSwerve andromedaSwerve;

  private RobotContainer() {
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }

  @Override
  public void initSubsystems() {
    andromedaSwerve = AndromedaSwerve.getInstance(new FalconAndromedaModule[] {
        new FalconAndromedaModule(0, "Front Right Module",
            AndromedaModuleConfig.getConfig(AndromedaProfiles.ANDROMEDA_CONFIG, AndromedaMap.mod1Const),
            DriveConstants.andromedaSwerveConfig),
        new FalconAndromedaModule(1, "Back Right Module",
            AndromedaModuleConfig.getConfig(AndromedaProfiles.ANDROMEDA_CONFIG, AndromedaMap.mod2Const),
            DriveConstants.andromedaSwerveConfig),
        new FalconAndromedaModule(2, "Back Left Module",
            AndromedaModuleConfig.getConfig(AndromedaProfiles.ANDROMEDA_CONFIG, AndromedaMap.mod3Const),
            DriveConstants.andromedaSwerveConfig),
        new FalconAndromedaModule(3, "Front Left Module",
            AndromedaModuleConfig.getConfig(AndromedaProfiles.ANDROMEDA_CONFIG, AndromedaMap.mod4Const),
            DriveConstants.andromedaSwerveConfig),
    }, DriveConstants.andromedaSwerveConfig);

  }

  @Override
  public void configureBindings() {
    andromedaSwerve.setDefaultCommand(
        new SwerveDriveCommand(
            andromedaSwerve,
            () -> -OperatorConstants.driverController1.getLeftX(),
            () -> -OperatorConstants.driverController1.getLeftY(),
            () -> -OperatorConstants.driverController1.getRightX(),
            () -> OperatorConstants.driverController1.leftStick().getAsBoolean()));
  }

}

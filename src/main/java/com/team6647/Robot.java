/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.andromedalib.andromedaSwerve.utils.LocalADStarAK;
import com.andromedalib.robot.SuperRobot;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.team6647.util.Constants.RobotConstants;

public class Robot extends SuperRobot {

  private RobotContainer container;

  @Override
  public void robotInit() {
    //Pathfinding.setPathfinder(new LocalADStarAK());

    System.out.println("[Init] Starting AdvantageKit");
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (RobotConstants.currentMode) {
      case REAL:
       Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
            "_sim")));
        break;
    }

    AutoLogOutputManager.addPackage("com.andromedalib");

    // Start AdvantageKit Logger, no more fields can be added
    Logger.start();

    container = RobotContainer.getInstance();
    super.setRobotContainer(container, false);
    super.robotInit();
  }
}

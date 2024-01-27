/**
 * Written by Juan Pablo Gutiérrez
 */
package com.andromedalib.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SuperRobot extends LoggedRobot {
    private SuperRobotContainer container;
    private BaseTelemetryManager telemetryManager;

    private Command autonomousCommand;

    private boolean useCamera;

    /**
     * Instantiates the {@link SuperRobotContainer}. Run this method before doing
     * anything else
     * 
     * @param container {@link SuperRobotContainer}
     */
    public void setRobotContainer(SuperRobotContainer container, BaseTelemetryManager telemetryManager,
            boolean useCamera) {
        this.telemetryManager = telemetryManager;
        this.container = container;
        this.useCamera = useCamera;
    }

    /**
     * Gets the {@link SuperRobotContainer} instance
     * 
     * @return {@link SuperRobotContainer} instance
     */
    public SuperRobotContainer getRobotContainer() {
        return container;
    }

    /**
     * Initiates all basic robot functions. Run super.robotInit() to run this code
     * if overriding
     */
    @Override
    public void robotInit() {
        container.initSubsystems();
        telemetryManager.initTelemetry();
        container.configureBindings();

        if (useCamera)
            CameraServer.startAutomaticCapture();
    }

    /**
     * Constantly runs while the robot is on. Run super.robotPeriodic() to run this
     * code
     * if overriding
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        telemetryManager.updateTelemetry();
    }

    /**
     * Runs when robot enters disabled. Run super.disabledInit() to run this code
     * if overriding
     */
    @Override
    public void disabledInit() {
    }

    /**
     * Constantly runs when robot is disabled. Run super.disabledPeriodic() to run
     * this code
     * if overriding
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * Runs when robot enters autonomous. Run super.autonomousInit() to run this
     * code
     * if overriding
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = container.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /**
     * Constantly runs when robot is in autonomous. Run super.autonomousPeriodic()
     * to run this code
     * if overriding
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * Runs when robot enters telep. Run super.teleopInit() to run this code
     * if overriding
     */
    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /**
     * Constantly runs when robot is in teleoo. Run super.teleopPeriodic() to run
     * this code
     * if overriding
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * Runs runs when robot enters test mode. Run super.testInit() to run this code
     * if overriding
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        Command testCommand = container.getTestCommand();

        if (testCommand != null) {
            testCommand.schedule();
        }
    }

    /**
     * Constantly runs when robot is in test mode. Run super.testPeriodic() to run
     * this code
     * if overriding
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * Runs when robot enters simualtion. Run super.simulationInit() to run this
     * code
     * if overriding
     */
    @Override
    public void simulationInit() {
    }

    /**
     * Constantly runs when robot is in simualtion. Run super.simulationPeriodi() to
     * run this code
     * if overriding
     */
    @Override
    public void simulationPeriodic() {
    }

}

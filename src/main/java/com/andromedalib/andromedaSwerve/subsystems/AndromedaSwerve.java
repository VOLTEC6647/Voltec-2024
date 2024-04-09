/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 22 03 2024
 */
package com.andromedalib.andromedaSwerve.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModule;
import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModuleIO;
import com.andromedalib.andromedaSwerve.andromedaModule.GyroIO;
import com.andromedalib.andromedaSwerve.andromedaModule.GyroIOInputsAutoLogged;
import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig;
import com.andromedalib.andromedaSwerve.utils.PhoenixOdometryThread;
import com.andromedalib.odometry.SuperRobotState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class AndromedaSwerve extends SubsystemBase {

  private AndromedaModule[] modules = new AndromedaModule[4];
  public AndromedaSwerveConfig andromedaProfile;

  private final SuperRobotState robotState;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  @AutoLogOutput(key = "Swerve/Rotation")
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4];

  public static final Lock odometryLock = new ReentrantLock();

  /* Characterization */
  private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.zero(Volts);
  private final MutableMeasure<Distance> m_distance = MutableMeasure.zero(Meters);
  private final MutableMeasure<Velocity<Distance>> m_velocity = MutableMeasure.zero(MetersPerSecond);

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (Measure<Voltage> volts) -> {
            runSwerveCharacterization(volts);
          },
          log -> {
            log.motor("drive-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        modules[3].getDriveVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(modules[3].getPosition().distanceMeters, Meters))
                .linearVelocity(
                    m_velocity.mut_replace(modules[3].getDriveSpeed(), MetersPerSecond));
            log.motor("drive-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        modules[0].getDriveVoltage() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(modules[3].getPosition().distanceMeters, Meters))
                .linearVelocity(
                    m_velocity.mut_replace(modules[0].getDriveSpeed(), MetersPerSecond));
          },
          this));

  /** Creates a new AndromedaSwerve. */
  public AndromedaSwerve(GyroIO gyroIO, AndromedaModuleIO[] modulesIO, AndromedaSwerveConfig andromedaProfile,
      SuperRobotState robotState) {
    this.andromedaProfile = andromedaProfile;
    this.robotState = robotState;
    modules[0] = new AndromedaModule(0, "Front Right", andromedaProfile, modulesIO[0]);
    modules[1] = new AndromedaModule(1, "Back Right", andromedaProfile, modulesIO[1]);
    modules[2] = new AndromedaModule(2, "Back Left", andromedaProfile, modulesIO[2]);
    modules[3] = new AndromedaModule(3, "Front Left", andromedaProfile, modulesIO[3]);
    lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    this.gyroIO = gyroIO;

    PhoenixOdometryThread.getInstance().start();
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Swerve/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Swerve/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Swerve/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      Logger.recordOutput("Swerve/DesiredChassisSpeeds", new ChassisSpeeds());
    }

    Logger.recordOutput("Swerve/SwerveStates/Measured", getModuleStates());

    Logger.recordOutput("Swerve/ChassisSpeeds", getRobotRelativeChassisSpeeds());

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    Logger.recordOutput("Swerve/Samplecount", sampleCount);
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = andromedaProfile.swerveKinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      robotState.addOdometryObservations(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
  }

  /**
   * Drives the robot with the given ChassisSpeeds 
   * 
   * @param chassisSpeeds The desired ChassisSpeeds
   */
  protected void drive(ChassisSpeeds chassisSpeeds) {

    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

    SwerveModuleState[] swerveModuleStates = andromedaProfile.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    Logger.recordOutput("Swerve/DesiredChassisSpeeds", chassisSpeeds);

    setModuleStates(swerveModuleStates);
  }

  /**
   * Gets Gyro Angle clamped to 0 - 360 degrees
   * 
   * @return
   */
  public Rotation2d getSwerveAngle() {
    return rawGyroRotation;
  }

  /**
   * Gets the heading velocity of the robot in degrees
   * 
   * @return Heading velocity
   */
  public double getHeadingVelocity() {
    return gyroInputs.yawVelocityDegrees;
  }

  /**
   * Sets the modules states
   * 
   * @param desiredStates Desired SwerveModuleState array
   * @param isOpenLoop    True if open loop driving
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, andromedaProfile.maxSpeed);

    Logger.recordOutput("Swerve/SwerveStates/Setpoints", desiredStates);

    for (AndromedaModule andromedaModule : modules) {
      andromedaModule.setDesiredState(desiredStates[andromedaModule.getModuleNumber()]);
    }
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "Swerve/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }

    return states;
  }

  /**
   * Returns the current {@link ChassisSpeeds} of the robot, relative to the robot
   * 
   * @return {@link ChassisSpeeds} of the robot
   */
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(andromedaProfile.swerveKinematics.toChassisSpeeds(getModuleStates()),
        getSwerveAngle());
  }

  /**
   * Returns the current {@link ChassisSpeeds} of the robot, relative to the field
   * 
   * @return {@link ChassisSpeeds} of the robot
   */
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return andromedaProfile.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns all module positions
   * 
   * @return SwerveModulePosition array
   */
  public SwerveModulePosition[] getPositions() {

    SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
    for (AndromedaModule andromedaModule : modules) {
      states[andromedaModule.getModuleNumber()] = andromedaModule.getPosition();
    }

    return states;
  }

  /**
   * Manually sets the gyro angle to a specified angle
   * 
   * @param angle New angle
   */
  public void setGyroAngle(Measure<Angle> angle) {
    gyroIO.setGyroAngle(Rotation2d.fromRotations(angle.in(Rotations)));
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement.
   * The modules will
   * return to their normal orientations the next time a nonzero velocity is
   * requested.
   */
  public void lock() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = andromedaProfile.moduleTranslations[i].getAngle();
    }
    SwerveModuleState[] states = andromedaProfile.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds());
    for (int i = 0; i < 4; i++) {
      states[i].angle = headings[i];
    }
    setModuleStates(states);
    andromedaProfile.swerveKinematics.resetHeadings(headings);
    stop();
  }

  /**
   * Stops the swerve drive
   */
  public void stop() {
    drive(new ChassisSpeeds());
  }

  /* Characterization */

  public void runSwerveCharacterization(Measure<Voltage> volts) {
    for (AndromedaModule andromedaModule : modules) {
      andromedaModule.runCharacterization(volts);
    }
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}

/**
 * Written by Juan Pablo Gutiérrez
 */
package com.andromedalib.andromedaSwerve.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModule;
import com.andromedalib.andromedaSwerve.andromedaModule.AndromedaModuleIO;
import com.andromedalib.andromedaSwerve.andromedaModule.GyroIO;
import com.andromedalib.andromedaSwerve.andromedaModule.GyroIOInputsAutoLogged;
import com.andromedalib.andromedaSwerve.config.AndromedaSwerveConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class AndromedaSwerve extends SubsystemBase {
  private static AndromedaSwerve instance;

  private AndromedaModule[] modules = new AndromedaModule[4];
  public AndromedaSwerveConfig andromedaProfile;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private SwerveDrivePoseEstimator poseEsimtator;

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (Measure<Voltage> volts) -> {
            runSwerveCharacterization(volts.in(Units.Volts));
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

  private AndromedaSwerve(GyroIO gyro, AndromedaModuleIO[] modulesIO, AndromedaSwerveConfig profileConfig) {
    this.andromedaProfile = profileConfig;
    modules[0] = new AndromedaModule(0, "Front Right", andromedaProfile, modulesIO[0]);
    modules[1] = new AndromedaModule(1, "Back Right", andromedaProfile, modulesIO[1]);
    modules[2] = new AndromedaModule(2, "Back Left", andromedaProfile, modulesIO[2]);
    modules[3] = new AndromedaModule(3, "Front Left", andromedaProfile, modulesIO[3]);

    this.gyroIO = gyro;

    poseEsimtator = new SwerveDrivePoseEstimator(profileConfig.swerveKinematics, getSwerveAngle(), getPositions(),
        new Pose2d());
  }

  public static AndromedaSwerve getInstance(GyroIO gyro, AndromedaModuleIO[] modules,
      AndromedaSwerveConfig profileConfig) {
    if (instance == null) {
      instance = new AndromedaSwerve(gyro, modules, profileConfig);
    }
    return instance;

  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      Logger.recordOutput("DesiredChassisSpeeds", new ChassisSpeeds());
    }
    Logger.recordOutput("SwerveStates/Measured", getModuleStates());

    Logger.recordOutput("ChassisSpeeds", getFieldRelativeChassisSpeeds());

    updateOdometry();
  }

  /**
   * Main driving method
   * 
   * @param translation   Translation movement
   * @param rotation      Desired rotation
   * @param fieldRelative True if field relative
   * @param isOpenLoop    True if open loop
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
            getSwerveAngle())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    drive(chassisSpeeds);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = andromedaProfile.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    Logger.recordOutput("DesiredChassisSpeeds", chassisSpeeds);

    setModuleStates(swerveModuleStates);

  }

  /* Telemetry */

  /**
   * Gets Navx Angle clamped to 0 - 360 degrees
   * 
   * @return
   */
  public Rotation2d getSwerveAngle() {
    return gyroInputs.yawPosition;
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }

    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEsimtator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose2d) {
    poseEsimtator.resetPosition(getSwerveAngle(), getPositions(), pose2d);
  }

  public void updateOdometry() {
    poseEsimtator.update(getSwerveAngle(), getPositions());
  }

  public void addVisionMeasurements(Pose2d visionMeasurement, double timestampSeconds) {
    poseEsimtator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  public ChassisSpeeds getRelativeChassisSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(andromedaProfile.swerveKinematics.toChassisSpeeds(getModuleStates()),
        getSwerveAngle());
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
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
   * Sets the modules states
   * 
   * @param desiredStates Desired SwerveModuleState array
   * @param isOpenLoop    True if open loop driving
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, andromedaProfile.maxSpeed);

    Logger.recordOutput("SwerveStates/Setpoints", desiredStates);

    for (AndromedaModule andromedaModule : modules) {
      andromedaModule.setDesiredState(desiredStates[andromedaModule.getModuleNumber()]);
    }
  }

  /* Characterization */

  public void runSwerveCharacterization(double volts) {
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

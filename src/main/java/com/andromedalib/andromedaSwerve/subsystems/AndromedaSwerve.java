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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
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

  private SwerveDrivePoseEstimator poseEstimator;

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final Vector<N3> stateStandardDeviations = VecBuilder.fill(0.03, 0.03,
      edu.wpi.first.math.util.Units.degreesToRadians(1));

  private final Vector<N3> visionmeasurementStandardDeviations = VecBuilder.fill(0.5, 0.5,
      edu.wpi.first.math.util.Units.degreesToRadians(50));

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

  private AndromedaSwerve(GyroIO gyro, AndromedaModuleIO[] modulesIO, AndromedaSwerveConfig profileConfig,
      HolonomicPathFollowerConfig pathConfig) {
    this.andromedaProfile = profileConfig;
    modules[0] = new AndromedaModule(0, "Front Right", andromedaProfile, modulesIO[0]);
    modules[1] = new AndromedaModule(1, "Back Right", andromedaProfile, modulesIO[1]);
    modules[2] = new AndromedaModule(2, "Back Left", andromedaProfile, modulesIO[2]);
    modules[3] = new AndromedaModule(3, "Front Left", andromedaProfile, modulesIO[3]);

    this.gyroIO = gyro;

    poseEstimator = new SwerveDrivePoseEstimator(profileConfig.swerveKinematics, new Rotation2d(),
        new SwerveModulePosition[] { new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition() },
        new Pose2d(), stateStandardDeviations, visionmeasurementStandardDeviations);

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getRelativeChassisSpeeds,
        this::drive,
        pathConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, this);
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  public static AndromedaSwerve getInstance(GyroIO gyro, AndromedaModuleIO[] modules,
      AndromedaSwerveConfig profileConfig, HolonomicPathFollowerConfig pathConfig) {
    if (instance == null) {
      instance = new AndromedaSwerve(gyro, modules, profileConfig, pathConfig);
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
      Logger.recordOutput("Swerve/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Swerve/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      Logger.recordOutput("Swerve/DesiredChassisSpeeds", new ChassisSpeeds());
    }
    Logger.recordOutput("Swerve/SwerveStates/Measured", getModuleStates());

    Logger.recordOutput("Swerve/ChassisSpeeds", getFieldRelativeChassisSpeeds());

    updateOdometry();
  }

  /**
   * Drives the robot with the given translation and rotation
   * 
   * @param translation   Translation movement
   * @param rotation      Desired rotation
   * @param fieldRelative True if field relative
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
            getSwerveAngle())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.2);

    drive(chassisSpeeds);
  }

  /**
   * Drives the robot with the given ChassisSpeeds 
   * 
   * @param chassisSpeeds The desired ChassisSpeeds
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = andromedaProfile.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    Logger.recordOutput("Swerve/DesiredChassisSpeeds", chassisSpeeds);

    setModuleStates(swerveModuleStates);
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
   * Locks the pose of the robot to all motors at 45 degrees to prevent movement
   */
  public void lockPose() {
    SwerveModuleState[] desiredStates = andromedaProfile.swerveKinematics
        .toSwerveModuleStates(new ChassisSpeeds(0, 0, 0.1));

    for (AndromedaModule andromedaModule : modules) {
      andromedaModule.setDesiredState(desiredStates[andromedaModule.getModuleNumber()]);
    }
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
  @AutoLogOutput(key = "Swerve/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }

    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Swerve/PoseEstimate/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose2d) {
    poseEstimator.resetPosition(getSwerveAngle(), getPositions(), pose2d);

  }

  public void updateOdometry() {
    poseEstimator.updateWithTime(Logger.getRealTimestamp(), getSwerveAngle(), getPositions());
  }

  public void addVisionMeasurements(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
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

  public Command getPathFindPath(Pose2d targetPose) {
    PathConstraints constraints = new PathConstraints(
        3, 4,
        edu.wpi.first.math.util.Units.degreesToRadians(560), edu.wpi.first.math.util.Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0,
        0.0);
  }
}

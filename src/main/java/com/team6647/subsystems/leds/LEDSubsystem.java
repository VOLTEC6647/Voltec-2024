/**
 * Credits: 6328
 * 
 * 07 03 2024
 */
package com.team6647.subsystems.leds;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private static LEDSubsystem instance;

  public static LEDSubsystem getInstance() {
    if (instance == null) {
      instance = new LEDSubsystem();
    }
    return instance;
  }

  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean requestAmp = false;
  public boolean intaking = false;
  public boolean intakeHasNote = false;
  public boolean shooterHasNote = false;
  public boolean autoShoot = false;
  public boolean autoDrive = false;
  public boolean climbing = false;
  public boolean trapping = false;
  public boolean endgameAlert = false;
  public boolean sameBattery = false;
  public boolean pivotHomed = true;
  public boolean armCoast = false;
  public boolean armEstopped = false;
  public boolean autoFinished = false;
  public double autoFinishedTime = 0.0;
  public boolean lowBatteryAlert = false;
  public boolean demoMode = false;

  private Optional<Alliance> alliance = Optional.empty();
  private Color allianceColor = Color.kGold;
  private Color secondaryDisabledColor = Color.kDarkBlue;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  // LED IO
  private final Spark leds = new Spark(0);
  private final Notifier loadingNotifier;

  // Constants
  private static final boolean prideLeds = false;
  private static final int minLoopCycleCount = 10;
  private static final int length = 12;
  private static final int staticSectionLength = 3;
  private static final double strobeFastDuration = 0.1;
  private static final double strobeSlowDuration = 0.2;
  private static final double breathDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveSlowCycleLength = 25.0;
  private static final double waveSlowDuration = 3.0;
  private static final double waveAllianceCycleLength = 15.0;
  private static final double waveAllianceDuration = 2.0;
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal

  private LEDSubsystem() {
    System.out.println("[Init] Creating LEDs");
    loadingNotifier = new Notifier(
        () -> {
          synchronized (this) {
            periodic();
          }
        });
    loadingNotifier.startPeriodic(0.02);
  }

  public synchronized void periodic() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      allianceColor = alliance
          .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kFirstRed)
          .orElse(Color.kGold);
      secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : Color.kDarkBlue;
    }

    // Update auto state
    if (DriverStation.isDisabled()) {
      autoFinished = false;
    } else {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getFPGATimestamp();
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    if (DriverStation.isDisabled()) {
      wave(); // Default to off
    }
    
    if (estopped) {
      solidRed();
    }

    if (endgameAlert) {
      strobeGreen();
    }

    if (intakeHasNote) {
      solidOrange();
    } else if (shooterHasNote) {
      solidGreen();
    } else if (!pivotHomed) {
      solidRed();
    } else {
      solidBlue();

    }
    // Update LEDs
  }

  private void solidRed() {
    leds.set(0.61);
  }

  private void solidBlue() {
    leds.set(0.83);
  }

  private void strobeGreen() {
    leds.set(-0.61);
  }

  private void solidGreen() {
    leds.set(0.73);
  }

  private void solidOrange() {
    leds.set(0.65);
  }

  private void breath() {
    leds.set(-0.15);
  }

  private void rainbow(double cycleLength, double duration) {
    leds.set(-0.93);
  }

  private void wave() {
    leds.set(-0.31);
  }

  public static boolean isRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return true;
  }
}

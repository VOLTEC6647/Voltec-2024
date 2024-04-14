/**
 * Credits: 6328
 * 
 * 07 03 2024
 */
package com.team6647.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
public class LEDSubsystem {
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
  public boolean pivotHomed = true;
  public boolean lowBatteryAlert = false;

  private boolean estopped = false;

  //private static TejuinoBoard leds = new TejuinoBoard();
  private final Notifier loadingNotifier;

  private static final int minLoopCycleCount = 10;

  private LEDSubsystem() {
    System.out.println("[Init] Creating LEDs");
    //leds.init(1);

    loadingNotifier = new Notifier(
        () -> {
          synchronized (this) {
            periodic();
          }
        });
    loadingNotifier.startPeriodic(0.02);

  }

  public synchronized void periodic() {
    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    if (estopped) {
      solidRed();
    }

    if (endgameAlert) {
      // strobeGreen();
    }

    /*
     * if(intakeHasNote){
     * strobeGreen(0.3);
     * }
     */
    // Update LEDs
  }

  public void strobeGreen(double duration) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    if (c1On) {
      solidGreen();
    } else {
      turnOffLeds();
    }
  }

  public void strobeYellow(double duration) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    if (c1On) {
      solidYellow();
    } else {
      turnOffLeds();
    }
  }

  public void solidYellow() {
    // leds.all_leds_yellow(1);
  }

  public void solidRed() {
    // leds.all_leds_red(1);
  }

  public void solidBlue() {
    // leds.all_leds_blue(1);
  }

  public void solidGreen() {
    // leds.all_leds_green(1);
  }

  public void turnOffLeds() {
    // leds.turn_off_all_leds(1);
  }

  public static boolean isRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return true;
  }
}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 20 02 2024
 */

package com.team6647.subsystems.neural;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team6647.RobotState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class NeuralVisionSubsystem extends SubsystemBase {

  private static NeuralVisionSubsystem instance;

  private NeuralVisionIO io;
  private NeuralVisionIOInputsAutoLogged inputs = new NeuralVisionIOInputsAutoLogged();

  public boolean isEnabled = false;

  public double lastTarget = 0;

  /** Creates a new NeuralVisionSubsystem. */
  private NeuralVisionSubsystem(NeuralVisionIO io) {
    this.io = io;
    //PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
  }

  public static NeuralVisionSubsystem getInstance(NeuralVisionIO io) {
    if (instance == null) {
      instance = new NeuralVisionSubsystem(io);
    }

    return instance;
  }

  //@Override
  public void periodic() {
    ////io.updateInputs(inputs);
    ////Logger.processInputs("Neural", inputs);
  }

  /* 
  //Todo: fix magic number p, fix time check amount, do offsets ig, might only need to call this once
  public Optional<Rotation2d> getRotationTargetOverride(){
    if(isEnabled&&DriverStation.isAutonomous()){
      if(inputs.hasTarget){
        lastTarget = Timer.getFPGATimestamp();
        return Optional.of(RobotState.getPose().getRotation().plus(Rotation2d.fromDegrees(10*inputs.TX)));
      }else if(Timer.getFPGATimestamp()-lastTarget<5){
        return(Optional.of(RobotState.getPose().getRotation()));
      }else{
        return Optional.empty();
      }
    }else{
      return Optional.empty();
    }
    
  }*/

  public boolean hasTarget() {
    return inputs.hasTarget;
  }

  public double getTY() {
    return inputs.TY;
  }

  public double getTX() {
    return inputs.TX;
  }

}

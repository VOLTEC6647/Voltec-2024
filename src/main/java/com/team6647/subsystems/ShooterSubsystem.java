// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647.subsystems;

import com.andromedalib.motorControllers.SuperTalonFX;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.ShooterConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem instance;
  private static NetworkTable shooterTable;
  private static StringEntry shooterStateEntry;
  private static BooleanEntry beamBrakeEntry;
  
  private SuperTalonFX shooterMotorLeft = new SuperTalonFX(ShooterConstants.shooterMotorLeftID, GlobalIdleMode.Coast, true);
  private SuperTalonFX shooterMotorRight = new SuperTalonFX(ShooterConstants.shooterMotorRightID, GlobalIdleMode.Coast, true);

  private RollerState mState = RollerState.STOPPED;
  private static DigitalInput beamBrake = new DigitalInput(ShooterConstants.beamBrakePort);

  private ShooterSubsystem(){

    shooterTable = NetworkTableInstance.getDefault().getTable("ShooterTable");
    shooterStateEntry = shooterTable.getStringTopic("ShooterState").getEntry(getRollerState().toString());
    beamBrakeEntry = shooterTable.getBooleanTopic("BeamBrake").getEntry(getBeamBrake());
  }

  public static ShooterSubsystem getInstance(){
    if(instance == null){
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  public enum RollerState {
    STOPPED, AMP, SPEAKER, TRAP
  }


  @Override 
  public void periodic() {
    updateNT();
  }

  public void changeRollerState(RollerState rollerState){

    switch(rollerState){

      case STOPPED:
        mState = RollerState.STOPPED;
        setShooterSpeed(ShooterConstants.passiveStopped);
        break;
      
      case AMP:
        mState = RollerState.AMP;
        setShooterSpeed(ShooterConstants.shooterSpeed);
        break;
      
      case SPEAKER:
        mState = RollerState.SPEAKER;
        setShooterSpeed(ShooterConstants.shooterSpeed);
        break;
      
      case TRAP:
        mState = RollerState.TRAP;
        setShooterSpeed(ShooterConstants.shooterSpeed);
        break;
    }
  }

  private void setShooterSpeed(double speed){
    shooterMotorRight.set(ControlMode.PercentOutput, speed);
  }

  private void updateNT(){
    shooterStateEntry.set(getRollerState().toString());
    beamBrakeEntry.set(getBeamBrake());
  }

  public RollerState getRollerState(){
    return mState;
  }

  public boolean getBeamBrake(){
    return beamBrake.get();
  }
}

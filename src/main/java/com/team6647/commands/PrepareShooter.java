package com.team6647.commands;

import com.team6647.subsystems.flywheel.ShooterSubsystem;
import com.team6647.subsystems.flywheel.ShooterSubsystem.FlywheelState;

import edu.wpi.first.wpilibj2.command.Command;

public class PrepareShooter extends Command{

    private ShooterSubsystem shooterSubsystem;

    public PrepareShooter(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;

        //addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        if(shooterSubsystem.mFlywheelState == FlywheelState.STOPPED){
            shooterSubsystem.setFlywheelState(FlywheelState.SHOOTING);
        }else if(shooterSubsystem.mFlywheelState == FlywheelState.SHOOTING){
            shooterSubsystem.setFlywheelState(FlywheelState.STOPPED);
        }
        
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}

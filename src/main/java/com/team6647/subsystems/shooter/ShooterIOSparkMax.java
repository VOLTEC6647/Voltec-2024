/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 22 01 2024
 */
package com.team6647.subsystems.shooter;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterIOSparkMax implements ShooterIO {

        private static SuperSparkMax topFlywheelMotor = new SuperSparkMax(
                        ShooterConstants.flywheelTopMotorID,
                        GlobalIdleMode.Coast,
                        true,
                        ShooterConstants.shooterMotorCurrentLimit);

        private static SuperSparkMax bottomFlywheelMotor = new SuperSparkMax(
                        ShooterConstants.flywheelBottomMotorID,
                        GlobalIdleMode.Coast,
                        true,
                        ShooterConstants.shooterMotorCurrentLimit);

        private SparkPIDController topFlywheelPID;
        private SparkPIDController bottomFlywheelPID;

        private static DigitalInput shooterBeamBrake = new DigitalInput(ShooterConstants.shooterBeamBrakeChannel);

        public ShooterIOSparkMax() {
                topFlywheelPID = topFlywheelMotor.getPIDController();
                bottomFlywheelPID = bottomFlywheelMotor.getPIDController();

                topFlywheelPID.setP(ShooterConstants.topShooterKp);
                topFlywheelPID.setI(ShooterConstants.topShooterKi);
                topFlywheelPID.setD(ShooterConstants.topShooterKd);
                topFlywheelPID.setFF(ShooterConstants.topShooterKf);

                bottomFlywheelPID.setP(ShooterConstants.shooterKp);
                bottomFlywheelPID.setI(ShooterConstants.shooterKi);
                bottomFlywheelPID.setD(ShooterConstants.shooterKd);
                bottomFlywheelPID.setFF(ShooterConstants.shooterKf);
        }

        @Override
        public void updateInputs(ShooterIOInputs inputs) {
                inputs.bottomMotorVelocity = bottomFlywheelMotor.getVelocity();
                inputs.topMotorVelocity = topFlywheelMotor.getVelocity();
                inputs.beamBrake = shooterBeamBrake.get();
        }

        @Override
        public void setShooterVelocity(double velocity) {
                topFlywheelPID.setReference(velocity, ControlType.kVelocity);
                bottomFlywheelPID.setReference(velocity, ControlType.kVelocity);
        }
}

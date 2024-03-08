/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 06 02 2024
 */

package com.team6647.subsystems.shooter.pivot;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.ShooterConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterPivotIOSparkMax implements ShooterPivotIO {

    private SuperSparkMax shooterPivotMotor = new SuperSparkMax(
            ShooterConstants.shooterPivotMotorID,
            GlobalIdleMode.Coast, ShooterConstants.shooterPivotMotorInverted,
            ShooterConstants.shooterMotorCurrentLimit,
            ShooterConstants.armEncoderPositionConversionFactor,
            ShooterConstants.armEncoderZeroOffset,
            ShooterConstants.armEncoderInverted);

    private static AbsoluteEncoder pivotEncoder;

    private static DigitalInput forwardLimitSwitch;

    private double setpoint;

    private double arbitraryFeedforward = 0;
    private double horizontalPosition = 112;
    private double maxGravityFF = ShooterConstants.pivotKf;

    ProfiledPIDController controller = new ProfiledPIDController(ShooterConstants.pivotKp, ShooterConstants.pivotKi,
            ShooterConstants.pivotKd, new TrapezoidProfile.Constraints(6000, 6000));

    public ShooterPivotIOSparkMax() {
        pivotEncoder = shooterPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

        shooterPivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ShooterConstants.pivotMaxPosition);
        shooterPivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ShooterConstants.pivotMinPosition);

        forwardLimitSwitch = new DigitalInput(ShooterConstants.forwardLimitSwitchID);
        
        controller.reset(pivotEncoder.getPosition());
    }

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        inputs.shooterAbsoluteEncoderPosition = pivotEncoder.getPosition();
        inputs.pivotMotorPosition = shooterPivotMotor.getPosition();
        inputs.shooterAbsoluteEncoderVelocity = pivotEncoder.getVelocity();

        inputs.shooterPivotAppliedVolts = shooterPivotMotor.getAppliedOutput()
                * shooterPivotMotor.getBusVoltage();

        inputs.inTolerance = Math
                .abs(pivotEncoder.getPosition() - this.setpoint) < ShooterConstants.positionTolerance;

        inputs.limitSwitchPressed = !forwardLimitSwitch.get();

        /* Apply a proportional force to the angle (to compensate for torque) */
        double degrees = (pivotEncoder.getPosition() - horizontalPosition);
        double radians = Units.degreesToRadians(degrees);
        double cosineScalar = Math.cos(radians);

        arbitraryFeedforward = cosineScalar * maxGravityFF;
        inputs.arbitraryFeedforward = arbitraryFeedforward;

        double output = controller.calculate(pivotEncoder.getPosition());

        output = output + arbitraryFeedforward;
        inputs.output = output;

        shooterPivotMotor.setVoltage(output);
    }

    @Override
    public void setShooterReference(double setpoint) {
        this.setpoint = setpoint;
        controller.setGoal(setpoint);
    }

    @Override
    public void setPIDF(double p, double i, double d, double f) {
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
        maxGravityFF = f;
    }

    @Override
    public void disablePivot() {
        shooterPivotMotor.disable();
        shooterPivotMotor.stopMotor();
    }
}

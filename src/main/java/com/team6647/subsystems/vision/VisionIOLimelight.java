/**
 * Written by Juan Pablo Gutiérrez
 */

package com.team6647.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import com.andromedalib.vision.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class VisionIOLimelight implements VisionIO {

    private Alliance alliance;

    // how many degrees back is your limelight rotated from perfectly vertical?
    private double limelightMountAngleDegrees = -30;

    // distance from the center of the Limelight lens to the floor
    private double limelightLensHeightMeters = 0.6;

    private double goalHeightMeters = 0.142;

    public VisionIOLimelight(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {

        inputs.targetDistance = computeDistance();

        LimelightHelpers.Results result = LimelightHelpers.getLatestResults("limelight").targetingResults;

        inputs.TA = LimelightHelpers.getTA("limelight");

        if (!(result.botpose[0] == 0 && result.botpose[1] == 0) &&
                LimelightHelpers.getTA("limelight") > 0.1) {
            inputs.hasTarget = true;

            inputs.observedPose2d = alliance == Alliance.Blue ? LimelightHelpers.toPose2D(result.botpose_wpiblue)
                    : LimelightHelpers.toPose2D(result.botpose_wpired);

            inputs.timestampLatency = Logger.getRealTimestamp()
                    - (result.latency_capture + result.latency_pipeline / 1000.0);

        } else {
            inputs.observedPose2d = new Pose2d();
            inputs.timestampLatency = 0;
            inputs.hasTarget = false;
        }
    }

    public double computeDistance() {
        double targetOffsetVertical = LimelightHelpers.getTY("limelight");

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetVertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // calculate distance
        return (goalHeightMeters - limelightLensHeightMeters)
                / Math.tan(angleToGoalRadians);
    }

}

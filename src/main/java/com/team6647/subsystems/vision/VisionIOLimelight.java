/**
 * Written by Juan Pablo Gutiérrez
 */

package com.team6647.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import com.andromedalib.vision.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class VisionIOLimelight implements VisionIO {

    private Alliance alliance;

    private Pose2d observedPose = new Pose2d();

    public VisionIOLimelight() {
        this.alliance = DriverStation.getAlliance().get();
        ;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        LimelightHelpers.Results result = LimelightHelpers.getLatestResults("limelight").targetingResults;

        inputs.TA = LimelightHelpers.getTA("limelight");

        if (!(result.botpose[0] == 0 && result.botpose[1] == 0) &&
                LimelightHelpers.getTA("limelight") < 30) {
            inputs.hasTarget = true;

            inputs.observedPose2d = alliance == Alliance.Blue ? LimelightHelpers.toPose2D(result.botpose_wpiblue)
                    : LimelightHelpers.toPose2D(result.botpose_wpired);

            inputs.timestampLatency = Logger.getRealTimestamp() - (result.latency_capture / 1000.0) -
                    (result.latency_pipeline / 1000.0);

        } else {
            inputs.observedPose2d = new Pose2d();
            inputs.timestampLatency = 0;
            inputs.hasTarget = false;
        }
    }

}

/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 25 02 2024
 */

package com.team6647.subsystems.vision;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionIOSim implements VisionIO {
    VisionSystemSim visionSim = new VisionSystemSim("main");

    TargetModel targetModel = TargetModel.kAprilTag36h11;

    AprilTagFieldLayout tagLayout;

    public VisionIOSim() {
        try {
            tagLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            visionSim.addAprilTags(tagLayout);

        } catch (Exception e) {
            DriverStation.reportError("Error while creating AprilTagFieldLayout", true);
        }
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
    }

}

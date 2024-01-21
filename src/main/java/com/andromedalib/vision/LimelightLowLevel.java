/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Wrapper for the {@link NetworkTable} class
 * that directly interacts with the Limelight's network table values.
 * Used by the {@link Limelight} class to get and set values.
 * 
 * Copied over and modified from
 * https://github.com/StuyPulse/StuyLib/blob/main/src/com/stuypulse/stuylib/network/limelight/LimelightTable.java
 */
public class LimelightLowLevel {
    /**
     * Global {@link NetworkTableInstance}
     */
    public final NetworkTableInstance tableInstance;

    /**
     * {@link NetworkTable} for the Limelight
     */
    public final NetworkTable table;

    /**
     * Set to 1 if the limelight has any valid targets (0 if no valid targets)
     */
    public final NetworkTableEntry validTarget;

    /**
     * Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
     */
    public final NetworkTableEntry targetX;

    /**
     * Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
     */
    public final NetworkTableEntry targetY;

    /**
     * Target Area (0% of image to 100% of image)
     */
    public final NetworkTableEntry targetArea;

    /**
     * Skew or rotation (-90 degrees to 0 degrees)
     */
    public final NetworkTableEntry targetSkew;

    /**
     * The pipeline’s latency contribution (ms) Add at least 11ms for image capture
     * latency.
     */
    public final NetworkTableEntry latency;

    /**
     * Capture pipeline latency (ms). Time between the end of the exposure of the
     * middle row of the sensor to the beginning of the tracking pipeline.
     */
    public final NetworkTableEntry captureLatency;

    /**
     * The shortest side of the fitted bounding box (pixels)
     */
    public final NetworkTableEntry shortSideLength;

    /**
     * The longest side of the fitted bounding box (pixels)
     */
    public final NetworkTableEntry longSideLength;

    /**
     * Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     */
    public final NetworkTableEntry horizontalLenght;

    /**
     * Vertical sidelength of the rough bounding box (0 - 320 pixels)
     */
    public final NetworkTableEntry verticalLength;

    /**
     * The index of the current pipeline (0 .. 9)
     */
    public final NetworkTableEntry pipelineIndex;

    /**
     * Value to set the pipeline to (0 .. 9)
     */
    public final NetworkTableEntry setPipeline;

    /**
     * Number array of corner X coordinates
     */
    public final NetworkTableEntry xCorners;

    /**
     * Number array of corner Y coordinates
     */
    public final NetworkTableEntry yCorners;

    /**
     * Result of 3D position solution: Translation (x,y,y) Rotation(pitch,yaw,roll)
     */
    public final NetworkTableEntry cam3DSol;

    /**
     * Sets limelight’s LED state
     * 0 = use the LED Mode set in the current pipeline
     * 1 = force off
     * 2 = force blink
     * 3 = force on
     */
    public final NetworkTableEntry ledMode;

    /**
     * Sets limelight’s operation mode
     * 0 = Vision Processor
     * 1 = Driver Camera (Increases exposure, disables vision processing)
     */
    public final NetworkTableEntry camMode;

    /**
     * Sets limelight’s streaming mode
     * 0 = Standard - Side-by-side streams if a webcam is attached to Limelight
     * 1 = PiP Main - The secondary camera stream is placed in the lower-right
     * corner of the primary camera stream
     * 2 = PiP Secondary - The primary camera stream is placed in the lower-right
     * corner of the secondary camera stream
     */
    public final NetworkTableEntry camStream;

    /**
     * Sets limelight to take a snapshot
     * 0 = Stop taking snapshots
     * 1 = Take two snapshots per second
     */
    public final NetworkTableEntry snapshot;

    /* AprilTags */

    public final NetworkTableEntry botpose;

    /**
     * Creates a new LimelightCamera object
     * 
     * @param name Name of the Limelight's network table
     */
    public LimelightLowLevel(String name) {
        tableInstance = NetworkTableInstance.getDefault();

        table = tableInstance.getTable(name);

        validTarget = table.getEntry("tv");

        targetX = table.getEntry("tx");

        targetY = table.getEntry("ty");

        targetArea = table.getEntry("ta");

        targetSkew = table.getEntry("ts");

        latency = table.getEntry("tl");

        captureLatency = table.getEntry("cl");

        shortSideLength = table.getEntry("tshort");

        longSideLength = table.getEntry("tlong");

        horizontalLenght = table.getEntry("thor");

        verticalLength = table.getEntry("tvert");

        pipelineIndex = table.getEntry("getpipe");

        setPipeline = table.getEntry("pipeline");

        xCorners = table.getEntry("tcornx");

        yCorners = table.getEntry("tcorny");

        cam3DSol = table.getEntry("camtran");

        ledMode = table.getEntry("ledMode");

        camMode = table.getEntry("camMode");

        camStream = table.getEntry("stream");

        snapshot = table.getEntry("snapshot");

        botpose = table.getEntry("botpose");
    }
}

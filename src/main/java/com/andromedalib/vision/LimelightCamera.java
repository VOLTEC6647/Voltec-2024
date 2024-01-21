/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.vision;

import com.andromedalib.math.Functions;

public class LimelightCamera {

    private final LimelightLowLevel lTable;

    /**
     * Create a new LimelightCamera object, with the default name of "limelight"
     */
    public LimelightCamera() {
        lTable = new LimelightLowLevel("limelight");
    }

    /**
     * Create a new LimelightCamera object
     * 
     * @param name Name of the Limelight Network Table
     */
    public LimelightCamera(String name) {
        lTable = new LimelightLowLevel(name);
    }

    /**
     * Checks if the Limelight is connected
     * 
     * @return True if the Limelight is connected, false otherwise
     */
    public boolean isConnected() {
        return lTable.tableInstance.isConnected();
    }

    /**
     * Gets the limelight's target information
     * 
     * @return Target information in a double array
     */
    public double[] getTargetInfo() {
        return new double[] { getX(), getY(), getArea(), getSkew(), getShortSideLength(), getLongSideLength(),
                getHorizontalLength(), getVerticalLength(), getLatency() };
    }

    /**
     * Get the current pipeline number
     * 
     * @return Current pipeline number
     */
    public double getPipeline() {
        return lTable.pipelineIndex.getDouble(0);
    }

    /**
     * Set the current pipeline number
     * 
     * @param pipeline Pipeline number to set
     */
    public void setPipeline(double pipeline) {
        lTable.setPipeline.setDouble(pipeline);
    }

    /**
     * Gets the target X offsets
     * 
     * @return Target X offsets
     */
    public double getX() {
        return lTable.targetX.getDouble(0);
    }

    /**
     * Gets the target Y offsets
     * 
     * @return Target Y offsets
     */
    public double getY() {
        return lTable.targetY.getDouble(0);
    }

    /**
     * Checks if the Limelight has any valid targets
     * 
     * @return True if the Limelight has any valid targets, false otherwise
     */
    public double hasValidTarget() {
        return lTable.validTarget.getDouble(0);
    }

    /**
     * Gets the target area percentage
     * 
     * @return Target area percentage
     */
    public double getArea() {
        return lTable.targetArea.getDouble(0);
    }

    /**
     * Gets the target skew (-90 to 0 degrees)
     * 
     * @return Target skew
     */
    public double getSkew() {
        return lTable.targetSkew.getDouble(0);
    }

    /**
     * Gets the target area percentage, clamped between 0 and 1
     * 
     * @return
     */
    public double getAreaClampPercent() {
        return Functions.clamp(lTable.targetArea.getDouble(0) / 100.0, 0, 1);
    }

    /**
     * Gets the shortest side of the fitted bounding box (pixels)
     * 
     * @return Shortest side of the fitted bounding box (pixels)F
     */
    public double getShortSideLength() {
        return lTable.shortSideLength.getDouble(0);
    }

    /**
     * Gets the longest side of the fitted bounding box (pixels)
     * 
     * @return Longest side of the fitted bounding box (pixels)
     */
    public double getLongSideLength() {
        return lTable.longSideLength.getDouble(0);
    }

    /**
     * Gets the vertical sidelength of the rough bounding box (0 - 320 pixels)
     * 
     * @return Vertical sidelength of the rough bounding box (0 - 320 pixels)
     */
    public double getHorizontalLength() {
        return lTable.horizontalLenght.getDouble(0);
    }

    /**
     * Gets the horizontal sidelength of the rough bounding box (0 - 320 pixels)
     * 
     * @return Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     */
    public double getVerticalLength() {
        return lTable.verticalLength.getDouble(0);
    }

    /**
     * Gets the pipeline latency (ms)
     * 
     * @return Pipeline latency (ms)
     */
    public double getLatency() {
        return lTable.latency.getDouble(0);
    }

    /**
     * Gets the pipeline capture latency
     * 
     * @return Capture latency
     */
    public double getCaptureLatency() {
        return lTable.captureLatency.getDouble(0);
    }

    /**
     * Gets the target's corner coordinates array. Returns empty array if the
     * lengths are not equal (should never happen)
     * 
     * @return Target's corner coordinates
     */
    public double[][] getAllCorners() {
        double[] xCorners = getXCorners();
        double[] yCorners = getYCorners();
        if (xCorners.length != yCorners.length)
            return new double[0][0];
        double[][] corners = new double[xCorners.length][yCorners.length];
        for (int i = 0; i < corners.length; i++) {
            corners[i][0] = xCorners[i];
            corners[i][1] = yCorners[i];
        }
        return corners;
    }

    /**
     * Gets the target's corner X coordinates array
     * 
     * @return Target's corner X coordinates
     */
    public double[] getXCorners() {
        return lTable.xCorners.getDoubleArray(new double[0]);
    }

    /**
     * Gets the target's corner Y coordinates array
     * 
     * @return Target's corner Y coordinates
     */
    public double[] getYCorners() {
        return lTable.yCorners.getDoubleArray(new double[0]);
    }

    /**
     * Gets the result of 3D position solution: Translation (x,y,y)
     * Rotation(pitch,yaw,roll)
     * 
     * @return Result of 3D position solutionF
     */
    public double getCam3DSol() {
        return lTable.cam3DSol.getDouble(0);
    }

    /**
     * Sets limelight’s LED state
     * 0 = use the LED Mode set in the current pipeline
     * 1 = force off
     * 2 = force blink
     * 3 = force on*
     * 
     * @param mode LED mode to set
     */
    public void setLedMode(int mode) {
        lTable.ledMode.setNumber(mode);
    }

    /**
     * Sets limelight’s cam mode
     * 0 = Vision Processor
     * 1 = Driver Camera (Increases exposure, disables vision processing)
     * 
     * @param mode Cam mode to set
     */
    public void setCamMode(int mode) {
        lTable.camMode.setNumber(mode);
    }

    /**
     * Sets limelight’s streaming mode
     * 0 = Standard - Side-by-side streams if a webcam is attached to Limelight
     * 1 = PiP Main - The secondary camera stream is placed in the lower-right
     * corner of the primary camera stream
     * 2 = PiP Secondary - The primary camera stream is placed in the lower-right
     * corner of the secondary camera stream
     * 
     * @param mode Stream mode to set
     */
    public void setStreamMode(int mode) {
        lTable.camStream.setNumber(mode);
    }

    /**
     * Sets limelight’s snapshot mode
     * 0 = Stop taking snapshots
     * 1 = Take two snapshots per second
     * 
     * @param mode Snapshot mode to set
     */
    public void setSnapshotMode(int mode) {
        lTable.snapshot.setNumber(mode);
    }

    /**
     * Geets a custom double value
     * 
     * @param key   Key of the value
     * @param value Default value
     * 
     * @return Value of the key search
     */
    public double getCustomDouble(String key, double value) {
        return lTable.table.getEntry(key).getDouble(value);
    }

    /**
     * Sets a custom double value
     * 
     * @param key   Key of the value
     * @param value Value to set
     */
    public void setCustomDouble(String key, double value) {
        lTable.table.getEntry(key).setDouble(value);
    }

    /**
     * Gets a custom boolean value
     * 
     * @param key Key of the value
     * 
     * @return Value of the key search
     */
    public boolean getCustomBoolean(String key) {
        return lTable.table.getEntry(key).getBoolean(false);
    }

    /**
     * Sets a custom boolean value
     * 
     * @param key   Key of the value
     * @param state State to set
     * 
     * @return Value of the key search
     */
    public void setCustomBoolean(String key, boolean state) {
        lTable.table.getEntry(key).setBoolean(state);
    }

    /**
     * Sets a custom number value
     * 
     * @param key   Key of the value
     * @param value Default value
     * 
     * @return Value of the key search
     */
    public void setCustomNumber(String key, double value) {
        lTable.table.getEntry(key).setNumber(value);
    }

    /**
     * Sets a custom string value
     * 
     * @param key Key of the value
     *            F
     */
    public void getCustomString(String key) {
        lTable.table.getEntry(key).getString("No value");
    }

    /**
     * Sets a custom string value
     * 
     * @param key   Key of the value
     * @param value Default value
     * 
     * @return Value of the key search
     */
    public void setCustomString(String key, String value) {
        lTable.table.getEntry(key).setString(value);
    }

    public double[] getBotpose() {
        return lTable.botpose.getDoubleArray(new double[7]);
    }

}

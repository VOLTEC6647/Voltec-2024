/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.shuffleboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public abstract class ShuffleboardTabBase {

    protected static String tabName;
    protected static ShuffleboardTab tab;

    /**
     * Adds data to the tab
     * 
     * @param name     Name of the data
     * @param sendable {@link Sendable} to be published to Shuffleboard
     */
    public void addData(String name, Sendable sendable) {
        tab.add(name, sendable);
    }

    /**
     * Adds double data to the tab
     * 
     * @param name       Name of the data
     * @param doubleData Double data to be published to Shuffleboard
     */
    public void addData(String name, double doubleData) {
        tab.add(name, doubleData);
    }

    /**
     * Adds double data to the tab with a selected width and height
     * 
     * @param name       Name of the data
     * @param doubleData Double data to be published to Shuffleboard
     * @param width      Width of the dat awidget
     * @param height     Height of the data widget
     */
    public void addData(String name, double doubleData, int width, int height) {
        tab.add(name, doubleData);
    }

    /**
     * Adds double data to the tab with a selected width, height x and y coordinates
     * 
     * @param name       Name of the data
     * @param doubleData Double data to be published to Shuffleboard
     * @param width      Width of the dat awidget
     * @param height     Height of the data widget
     * @param x          X widget position
     * @param y          Y widget position
     */
    public void addData(String name, double doubleData, int width, int height, int x, int y) {
        tab.add(name, doubleData);
    }

    /**
     * Adds data to the tab with a selected width and height
     * 
     * @param name     Name of the data
     * @param sendable {@link Sendable} to be published to Shuffleboard
     * @param width    Width of the data widget
     * @param height   Height of the data widget
     */
    public void addData(String name, Sendable sendable, int width, int height) {
        tab.add(name, sendable).withSize(width, height);
    }

    /**
     * Adds data to the tab with a selected width, height and position
     * 
     * @param name     Name of the data
     * @param sendable {@link Sendable} to be published to Shuffleboard
     * @param width    Width of the data widget
     * @param height   Height of the data widget
     * @param x        X widget position
     * @param y        Y widget position
     */
    public void addData(String name, Sendable sendable, int width, int height, int x, int y) {
        tab.add(name, sendable).withSize(width, height).withPosition(x, y);
    }

    /**
     * Updates published telemetry
     */
    public abstract void updateTelemetry();
}

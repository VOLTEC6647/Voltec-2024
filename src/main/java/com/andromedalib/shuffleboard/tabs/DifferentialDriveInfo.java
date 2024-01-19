/**
 * Written by Juan Pablo Gutiérrez
 */

package com.andromedalib.shuffleboard.tabs;

import com.andromedalib.shuffleboard.ShuffleboardTabBase;
import com.andromedalib.subsystems.DifferentialDriveSubsystem;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DifferentialDriveInfo extends ShuffleboardTabBase {

    DifferentialDriveSubsystem chassis;

    ShuffleboardTab tab;

    GenericEntry drive;
    GenericEntry leftSpeed;
    GenericEntry rightSpeed;

    /**
     * Creates a DifferentialDriveInfo into a selectedTab
     * 
     * @param tab Shuffleboard tab on which to publish info
     */
    public DifferentialDriveInfo(ShuffleboardTab tab, DifferentialDriveSubsystem chassis) {
        this.chassis = chassis;
        rightSpeed = tab.add("Right Drive Speed", 0.0).withPosition(3, 0) .getEntry();
        leftSpeed = tab.add("Left Drive Speed", 0.0).withPosition(4,0).getEntry();
        tab.add("Differential Drive", chassis.getDrive()).withPosition(0, 0).withWidget(BuiltInWidgets.kDifferentialDrive);
    }

    /**
     * Updates telemetry info. Call this method manually.
     */
    public void updateTelemetry() {
        leftSpeed.setDouble(chassis.getLeftSpeed());
        rightSpeed.setDouble(chassis.getRightSpeed());
    }
}

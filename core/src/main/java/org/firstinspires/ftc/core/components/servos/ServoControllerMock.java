/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Controller managing mock servos
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.servos;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.ServoController;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class ServoControllerMock implements ServoControllerComponent {

    final LogManager            mLogger;

    final boolean               mConfigurationValid;

    ServoController.PwmStatus   mStatus;


    /* -------------- Constructors --------------- */
    public ServoControllerMock( LogManager logger)
    {
        mConfigurationValid = true;
        mLogger             = logger;
        mStatus             = ServoController.PwmStatus.DISABLED;
    }

    /* --------------------- Custom functions ---------------------- */

    /**
     * Determines if the mock servo controller component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /* ------------------ HardwareDevice functions ----------------- */

    /**
     * Returns an indication of the manufacturer of this device.
     * @return the manufacturer
     */
    @Override
    public Manufacturer                 getManufacturer() { return Manufacturer.Other; }

    /**
     * Returns a string suitable for display to the user as to the type of device.Note that this is a device-type-specific name; it has nothing to do with the name by which a user might have configured the device in a robot configuration.
     * @return the device name
     */
    @Override
    public String                       getDeviceName() { return "Servo Mock"; }

    /**
     * Get connection information about this device in a human readable format
     * @return connection information
     */
    @Override
    public String                       getConnectionInfo() { return ""; }

    /**
     * Version
     */
    @Override
    public int                          getVersion() { return 0; }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void                         resetDeviceConfigurationForOpMode() {}

    /**
     * Closes this device
     */
    @Override
    public void                         close() {}


    /* ----------------- ServoController functions ----------------- */

    /**
     * Enables all of the servos connected to this controller
     */
    @Override
    public void	                        pwmEnable(){
        mStatus = ServoController.PwmStatus.ENABLED;
    }

    /**
     * Disables all of the servos connected to this controller
     */
    @Override
    public void	                        pwmDisable(){
        mStatus = ServoController.PwmStatus.DISABLED;
    }

    /**
     * Returns the enablement status of the collective set of servos connected to this controller
     * @return the enablement status of the collective set of servos connected to this controller
     */
    @Override
    public ServoController.PwmStatus	getPwmStatus() { return mStatus; }


    @Override
    public void                         setServoPosition(int servo, double position) {}

    @Override
    public double	                    getServoPosition(int servo) { return -1;}

}

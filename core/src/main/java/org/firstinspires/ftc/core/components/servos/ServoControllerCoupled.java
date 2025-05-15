/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Coupled Controller managing coupled servos together
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.servos;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.ServoController;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class ServoControllerCoupled implements ServoControllerComponent {

    final LogManager        mLogger;

    boolean                 mConfigurationValid;

    final String            mName;

    final ServoController   mFirst;
    final ServoController   mSecond;

    /* -------------- Constructors --------------- */
    public ServoControllerCoupled(ServoController first, ServoController second, String name, LogManager logger)
    {
        mConfigurationValid  = true;

        mLogger = logger;

        mName   = name;

        mFirst  = first;
        mSecond = second;

        if(mFirst  == null) { mConfigurationValid = false; }
        if(mSecond == null) { mConfigurationValid = false; }

        if(mConfigurationValid && mFirst.equals(mSecond)) {
            // If coupled servos have the same controller, it won't be possible to power one
            // without powering the other. It won't be possible to pilot them separately and
            // check if coupling won't destroy them.
            mLogger.warning("Coupled servos " + mName + " have same controller.");
        }
    }

    /* --------------------- Custom functions ---------------------- */

    /**
     * Determines if the coupled servo controller component is configured correctly.
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
    public Manufacturer                 getManufacturer() {
        Manufacturer result = Manufacturer.Unknown;
        if(mConfigurationValid) {
            result = mFirst.getManufacturer();
        }
        return result;
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.Note that this is a device-type-specific name; it has nothing to do with the name by which a user might have configured the device in a robot configuration.
     * @return the device name
     */
    @Override
    public String                       getDeviceName() {
        String result = "";
        if(mConfigurationValid) {
            result = mFirst.getDeviceName() + " coupled with " + mSecond.getDeviceName();
        }
        return result;
    }

    /**
     * Get connection information about this device in a human readable format
     * @return connection information
     */
    @Override
    public String                       getConnectionInfo() {
        String result = "";
        if(mConfigurationValid) {
            result = "First : " + mFirst.getConnectionInfo();
            result += "\nSecond : " + mSecond.getConnectionInfo();
        }
        return result;
    }

    /**
     * Version
     */
    @Override
    public int                          getVersion() {
        int result = -1;
        if(mConfigurationValid) {
            result = mFirst.getVersion();
        }
        return result;
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void                         resetDeviceConfigurationForOpMode() {
        if(mConfigurationValid) {
            mFirst.resetDeviceConfigurationForOpMode();
            mSecond.resetDeviceConfigurationForOpMode();
        }
    }

    /**
     * Closes this device
     */
    @Override
    public void                         close()
    {
        if(mConfigurationValid) {
            mFirst.close();
            mSecond.close();
        }
    }

    /* ----------------- ServoController functions ----------------- */

    /**
     * Enables all of the servos connected to this controller
     */
    @Override
    public void	                        pwmEnable(){
        if(mConfigurationValid) {
            mFirst.pwmEnable();
            mSecond.pwmDisable();
        }
    }

    /**
     * Disables all of the servos connected to this controller
     */
    @Override
    public void	                        pwmDisable(){
        if(mConfigurationValid) {
            mFirst.pwmDisable();
            mSecond.pwmDisable();
        }
    }

    /**
     * Returns the enablement status of the collective set of servos connected to this controller
     * @return the enablement status of the collective set of servos connected to this controller
     */
    @Override
    public ServoController.PwmStatus	getPwmStatus(){
        ServoController.PwmStatus result = ServoController.PwmStatus.DISABLED;
        if(mConfigurationValid) {
            result = mFirst.getPwmStatus();
        }
        return result;
    }

    @Override
    public void                         setServoPosition(int servo, double position) {}

    @Override
    public double	                    getServoPosition(int servo) { return -1;}

}

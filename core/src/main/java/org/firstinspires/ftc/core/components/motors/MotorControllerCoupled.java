/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Coupled Controller managing coupled servos together
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.motors;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class MotorControllerCoupled implements MotorControllerComponent {

    final LogManager        mLogger;

    boolean                 mConfigurationValid;

    final String            mName;

    final DcMotorController   mFirst;
    final DcMotorController   mSecond;

    /* -------------- Constructors --------------- */
    public MotorControllerCoupled(DcMotorController first, DcMotorController second, String name, LogManager logger)
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
     * Returns a string suitable for display to the user as to the type of device.Note that this is a device-type-specific name; it has nothing to do with thename by which a user might have configured the device in a robot configuration.
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

    /* ---------------- DcMotorController functions ---------------- */

    @Override
    public MotorConfigurationType       getMotorType(int motor) { return null; }
    @Override
    public DcMotor.RunMode	            getMotorMode(int motor) { return DcMotor.RunMode.RUN_WITHOUT_ENCODER; }
    @Override
    public double	                    getMotorPower(int motor) { return 0; }
    @Override
    public boolean	                    isBusy(int motor) { return false; }
    @Override
    public DcMotor.ZeroPowerBehavior	getMotorZeroPowerBehavior(int motor) { return DcMotor.ZeroPowerBehavior.UNKNOWN; }
    @Override
    public boolean	                    getMotorPowerFloat(int motor) { return false; }
    @Override
    public int                          getMotorTargetPosition(int motor) { return 0; }
    @Override
    public int	                        getMotorCurrentPosition(int motor) { return 0; }

    @Override
    public void                         setMotorType(int motor, MotorConfigurationType motorType) {}
    @Override
    public void	                        setMotorMode(int motor, DcMotor.RunMode mode) {}
    @Override
    public void	                        setMotorPower(int motor, double power) {}
    @Override
    public void	                        setMotorZeroPowerBehavior(int motor, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {}
    @Override
    public void	                        setMotorTargetPosition(int motor, int position) {}
    @Override
    public void	                        resetDeviceConfigurationForOpMode(int motor) {}
}

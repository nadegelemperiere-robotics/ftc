/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * VoltageSensorComponent provides centralized initialization and
 * configuration management for the built-in IMU sensor
 * in FTC robots.
 * -------------------------------------------------------
 * This class facilitates the initialization and handling
 * of the built-in IMU component within the FTC Control Hub,
 * allowing for easy configuration using JSON input. It
 * provides methods for reading sensor orientation,
 * managing configuration states, and logging system
 * parameters.
 * <p>
 * Features:
 * - Initializes and configures an IMU from a JSON-based
 *   configuration file.
 * - Provides access to heading and heading velocity
 *   values in radians.
 * - Manages and applies heading offsets for calibration.
 * - Supports logging of configuration and status details.
 * <p>
 * Dependencies:
 * - Qualcomm Robotics SDK
 * - FTC SDK
 * - JSON Processing (org.json)
 * - Custom LogManager for logging
 * <p>
 * Usage:
 * 1. Create an instance of VoltageSensorComponent with the robot's
 *    hardware map and logger.
 * 2. Configure the IMU by reading a JSON configuration.
 * 3. Retrieve IMU orientation and angular velocity as
 *    needed.
 * <p>
 * Example:
 * {@code
 *      JSONObject config = new JSONObject();
 *      config.put("hwmap", "imu");
 *      config.put("logo-direction", "up");
 *      config.put("usb-direction", "right");
 * <p>
 *      VoltageSensorComponent imu = new VoltageSensorComponent("imu", hardwareMap, logger);
 *      imu.read(config);
 *      double heading = imu.heading();
 * }
 */

package org.firstinspires.ftc.core.components.voltage;

/* JSON includes */
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class VoltageSensorBuiltIn implements VoltageSensorComponent {

    public static final String  sTypeKey    = "built-in";

    final LogManager            mLogger;

    boolean                     mConfigurationValid;
    final String                mName;

    final HardwareMap           mMap;
    VoltageSensor               mVoltageSensor;


    /* ----------------------- Constructors ------------------------ */
    /**
     * Creates an VoltageSensorComponent instance with a specified name, hardware map, and logger.
     *
     * @param name The unique identifier for the IMU component.
     * @param hwMap The FTC HardwareMap to retrieve the IMU hardware.
     * @param logger The logging manager to handle system logs.
     */
    public VoltageSensorBuiltIn(String name, HardwareMap hwMap, LogManager logger) {
        mConfigurationValid = false;
        mLogger             = logger;
        mName               = name;
        mMap                = hwMap;
        if(mMap != null) {
            mVoltageSensor      = mMap.voltageSensor.iterator().next();
            mConfigurationValid = true;
        }
    }


    /**
     * Retrieves the name of the IMU component.
     *
     * @return The name of the component.
     */
    public String                       getName() { return mName; }

    /**
     * Logs the current imu yaw, pitch and roll.
     *
     * @return A formatted string containing imu telemetry data.
     */
    @Override
    public String                       log() {
        String result = "";
        if(mConfigurationValid) {
            result += "\n Voltage : " + mVoltageSensor.getVoltage();
        }
        return result;
    }

    /* ------------------ Configurable functions ------------------- */

    /**
     * Determines if the IMU component has been properly configured.
     *
     * @return True if configuration is valid, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid; }

    /**
     * Reads the IMU configuration from a JSON object and initializes the IMU.
     *
     * @param reader The JSON object containing configuration settings.
     */
    @Override
    public void                         read(JSONObject reader) {
    }

    /**
     * Writes the current IMU configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) {
    }

    /**
     * Generates an HTML representation of the IMU configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted IMU configuration.
     */
    @Override
    public String                       logConfigurationHTML() {
        StringBuilder result = new StringBuilder();
        return result.toString();
    }

    /**
     * Generates a text-based representation of the IMU configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted IMU configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {
        StringBuilder result = new StringBuilder();
        return result.toString();
    }

    /* ------------------ HardwareDevice functions ----------------- */

    /**
     * Returns an indication of the manufacturer of this device.
     * @return the manufacturer
     */
    @Override
    public HardwareDevice.Manufacturer getManufacturer()
    {
        HardwareDevice.Manufacturer result = HardwareDevice.Manufacturer.Unknown;
        if(mConfigurationValid) {
            result = mVoltageSensor.getManufacturer();
        }
        return result;
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.Note that this is a device-type-specific name; it has nothing to do with thename by which a user might have configured the device in a robot configuration.
     * @return the device name
     */
    @Override
    public String                       getDeviceName()
    {
        String result = "";
        if(mConfigurationValid) {
            result = mVoltageSensor.getDeviceName();
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
            result = mVoltageSensor.getConnectionInfo();
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
            result = mVoltageSensor.getVersion();
        }
        return result;
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void                         resetDeviceConfigurationForOpMode() {
        if(mConfigurationValid) {
            mVoltageSensor.resetDeviceConfigurationForOpMode();
        }
    }

    /**
     * Closes this device
     */
    @Override
    public void                         close()
    {
        if(mConfigurationValid) {
            mVoltageSensor.close();
        }
    }

    /* ------------------- Voltage Sensor methods override ------------------ */

    public double                       getVoltage()
    {
        double result = 0;
        if(mConfigurationValid) {
            result = mVoltageSensor.getVoltage();
        }
        return result;
    }
}
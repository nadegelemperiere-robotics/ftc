/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * ImuComponent provides centralized initialization and
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
 * 1. Create an instance of ImuComponent with the robot's
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
 *      ImuComponent imu = new ImuComponent("imu", hardwareMap, logger);
 *      imu.read(config);
 *      double heading = imu.heading();
 * }
 */

package org.firstinspires.ftc.core.components.imus;

/* JSON includes */
import org.json.JSONObject;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class ImuMock implements ImuComponent {

    public static final String sTypeKey    = "mock";
    final LogManager                                mLogger;

    final boolean                                   mConfigurationValid;
    final String                                    mName;

    double                                          mCurrentHeading;

    /* ----------------------- Constructors ------------------------ */
    /**
     * Creates an ImuComponent instance with a specified name, hardware map, and logger.
     *
     * @param name The unique identifier for the IMU component.
     * @param logger The logging manager to handle system logs.
     */
    public ImuMock(String name, LogManager logger) {

        mConfigurationValid  = true;
        mLogger = logger;
        mName   = name;
        mCurrentHeading = 0;
    }

    /**
     * Retrieves the name of the IMU component.
     *
     * @return The name of the component.
     */
    public String                       name() { return mName; }

    /**
     * Cache current imu value to enable multiple calls in a loop without
     */
    public void                         update() {}

    /**
     * Yaw reste function
     */
    public void                         reset() {
        mCurrentHeading = 0;
    }

    /**
     * Logs the current imu yaw, pitch and roll.
     *
     * @return A formatted string containing imu telemetry data.
     */
    public String                       log() {
        String result = "";
        if(mConfigurationValid) {
            result += "\n Y : " + 0 + " P : " + 0 + " R : " + 0;
        }
        return result;
    }

    /**
     * Retrieves the current heading (yaw angle) of the robot in radians.
     *
     * @return The heading in radians, adjusted with the configured offset.
     */
    public double                       heading() { return mCurrentHeading; }

    /**
     * Retrieves the current angular velocity around the Z-axis in radians per second.
     *
     * @return The IMU's angular velocity in radians per second.
     */
    public double                       headingVelocity() { return 0; }

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
    public void                         read(JSONObject reader) { }

    /**
     * Writes the current IMU configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) { }

    /**
     * Generates an HTML representation of the IMU configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted IMU configuration.
     */
    @Override
    public String                       logConfigurationHTML() { return "<p>Mock</p>\n"; }

    /**
     * Generates a text-based representation of the IMU configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted IMU configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {  return header + "> Mock\n"; }
}
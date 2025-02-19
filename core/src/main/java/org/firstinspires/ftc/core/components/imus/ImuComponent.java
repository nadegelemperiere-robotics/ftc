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

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;


public interface ImuComponent extends Configurable {

     /**
     * Factory method to create and configure an ImuComponent from a JSON object.
     *
     * @param name The name of the IMU component.
     * @param reader A JSON object containing the configuration parameters.
     * @param map The FTC HardwareMap to retrieve the IMU hardware.
     * @param logger The logging manager to handle system logs.
     * @return A fully configured ImuComponent instance.
     */
    static ImuComponent factory(String name, JSONObject reader, HardwareMap map, LogManager logger) {

        ImuComponent result = null;

        switch (name) {
            case "built-in":
                result = new ImuBuiltIn(name, map, logger);
                result.read(reader);
                break;
            case "mock":
                result = new ImuMock(name, logger);
                result.read(reader);
                break;
        }

        return result;
    }

    /* --------------------- Custom functions ---------------------- */

    String                      name();
    String                      log();

    /* ------------------ Configurable functions ------------------- */

    void                        read(JSONObject reader);
    void                        write(JSONObject writer);
    boolean                     isConfigured();
    String                      logConfigurationHTML();
    String                      logConfigurationText(String header);

    /* ------------------------ Imu functions ----------------------- */

    double                      heading();
    double                      headingVelocity();



}
/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * ImuComponent Interface
 * -------------------------------------------------------
 * The ImuComponent interface defines the contract for
 * managing and configuring IMU (Inertial Measurement Unit)
 * sensors in FTC robots. It provides methods for reading
 * sensor orientation, managing configuration states, and
 * logging system parameters.
 * -------------------------------------------------------
 * Features:
 * - Factory method to create and configure IMU components
 *   (e.g., built-in IMU or mock IMU) from JSON input.
 * - Provides access to heading and heading velocity values
 *   in radians.
 * - Supports resetting and calibrating the IMU heading.
 * - Manages configuration states and logs configuration
 *   details in HTML or text format.
 * -------------------------------------------------------
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
            case ImuBuiltIn.sTypeKey:
                result = new ImuBuiltIn(name, map, logger);
                result.read(reader);
                break;
            case ImuMock.sTypeKey:
                result = new ImuMock(name, logger);
                result.read(reader);
                break;
        }

        return result;
    }

    /* --------------------- Custom functions ---------------------- */

    String                      name();
    String                      log();
    void                        update();

    /* ------------------ Configurable functions ------------------- */

    void                        read(JSONObject reader);
    void                        write(JSONObject writer);
    boolean                     isConfigured();
    String                      logConfigurationHTML();
    String                      logConfigurationText(String header);

    /* ------------------------ Imu functions ----------------------- */

    double                      heading();
    double                      headingVelocity();
    void                        reset();



}
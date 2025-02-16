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

/* System includes */
import java.util.Map;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller */
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* Configuration includes */
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;


public class ImuComponent implements Configurable {

    static final String sHwMapKey          = "hwmap";
    static final String sLogoDirectionKey  = "logo-direction";
    static final String sUsbDirectionKey   = "usb-direction";

    Map<String, RevHubOrientationOnRobot.LogoFacingDirection> sString2LogoFacing = Map.of(
            "up", RevHubOrientationOnRobot.LogoFacingDirection.UP,
            "down",RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
            "right",RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            "left",RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            "backward",RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
            "forward",RevHubOrientationOnRobot.LogoFacingDirection.FORWARD
    );
    Map<RevHubOrientationOnRobot.LogoFacingDirection, String> sLogoFacing2String = Map.of(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,      "up",
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN,    "down",
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,   "right",
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,    "left",
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,"backward",
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, "forward"
    );

    Map<String, RevHubOrientationOnRobot.UsbFacingDirection> sString2UsbFacing = Map.of(
            "up", RevHubOrientationOnRobot.UsbFacingDirection.UP,
            "down",RevHubOrientationOnRobot.UsbFacingDirection.DOWN,
            "right",RevHubOrientationOnRobot.UsbFacingDirection.RIGHT,
            "left",RevHubOrientationOnRobot.UsbFacingDirection.LEFT,
            "backward",RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD,
            "forward",RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    );
    Map<RevHubOrientationOnRobot.UsbFacingDirection, String> sUsbFacing2String = Map.of(
            RevHubOrientationOnRobot.UsbFacingDirection.UP,         "up",
            RevHubOrientationOnRobot.UsbFacingDirection.DOWN,       "down",
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT,      "right",
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT,       "left",
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD,   "backward",
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,"forward"
    );

    /**
     * Factory method to create and configure an ImuComponent from a JSON object.
     *
     * @param name The name of the IMU component.
     * @param reader A JSON object containing the configuration parameters.
     * @param map The FTC HardwareMap to retrieve the IMU hardware.
     * @param logger The logging manager to handle system logs.
     * @return A fully configured ImuComponent instance.
     */
    public static ImuComponent factory(String name, JSONObject reader, HardwareMap map, LogManager logger) {

        ImuComponent result = new ImuComponent(name, map, logger);
        result.read(reader);
        return result;
    }

    LogManager                  mLogger;

    boolean                     mConfigurationValid;
    String                      mName;
    String                      mHwName;

    HardwareMap                 mMap;
    IMU                         mImu;
    RevHubOrientationOnRobot.LogoFacingDirection mLogo;
    RevHubOrientationOnRobot.UsbFacingDirection mUsb;


    double                      mHeadingOffset;

    /* ----------------------- Constructors ------------------------ */
    /**
     * Creates an ImuComponent instance with a specified name, hardware map, and logger.
     *
     * @param name The unique identifier for the IMU component.
     * @param hwMap The FTC HardwareMap to retrieve the IMU hardware.
     * @param logger The logging manager to handle system logs.
     */
    public ImuComponent(String name, HardwareMap hwMap, LogManager logger) {
        mConfigurationValid  = false;
        mLogger = logger;
        mName   = name;
        mHwName = "";
        mMap    = hwMap;
        mImu = null;
        mHeadingOffset = 0;
    }

    /**
     * Retrieves the configured heading offset.
     *
     * @return The current heading offset in radians.
     */
    public double                       headingOffset() { return mHeadingOffset; }

    /**
     * Sets the heading offset for the IMU.
     *
     * @param offset The heading offset in radians.
     */
    public void                         headingOffset(double offset) { mHeadingOffset = offset; }

    /**
     * Retrieves the name of the IMU component.
     *
     * @return The name of the component.
     */
    public String                       getName() { return mName; }

    /**
     * Retrieves the current heading (yaw angle) of the robot in radians.
     *
     * @return The heading in radians, adjusted with the configured offset.
     */
    public double                       heading() {
        double result = mImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        result += mHeadingOffset;
        return result;
    }

    /**
     * Retrieves the current angular velocity around the Z-axis in radians per second.
     *
     * @return The IMU's angular velocity in radians per second.
     */
    public double                       headingVelocity() {
        AngularVelocity velocity = mImu.getRobotAngularVelocity(AngleUnit.RADIANS);
        return velocity.zRotationRate;
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
        mConfigurationValid = false;
        mImu = null;

        try {
            if(reader.has(sHwMapKey)) {
                mHwName = reader.getString(sHwMapKey);
                mImu = mMap.tryGet(IMU.class, mHwName);
            }

            if(mImu != null && reader.has(sLogoDirectionKey) && reader.has(sUsbDirectionKey)) {
                if(sString2LogoFacing.containsKey(reader.getString(sLogoDirectionKey))) {
                    mLogo = sString2LogoFacing.get(reader.getString(sLogoDirectionKey));
                }
                if(sString2UsbFacing.containsKey(reader.getString(sUsbDirectionKey))) {
                    mUsb = sString2UsbFacing.get(reader.getString(sUsbDirectionKey));
                }

                RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(mLogo, mUsb);
                mImu.initialize(new IMU.Parameters(orientation));
                mImu.resetYaw();

                mConfigurationValid = true;
            }
        } catch(JSONException e) {
            mLogger.error(e.getMessage());
        }

        if(mImu == null) { mConfigurationValid = false; }
    }

    /**
     * Writes the current IMU configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) {
        if(mConfigurationValid) {
            try {
                writer.put(sHwMapKey, mHwName);
                writer.put(sLogoDirectionKey, sLogoFacing2String.get(mLogo));
                writer.put(sUsbDirectionKey, sUsbFacing2String.get(mUsb));
            } catch (JSONException e) {
                mLogger.error(e.getMessage());
            }
        }
    }

    /**
     * Generates an HTML representation of the IMU configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted IMU configuration.
     */
    @Override
    public String                       logConfigurationHTML() {
        StringBuilder result = new StringBuilder();
        if (mConfigurationValid) {
            result.append("<p style=\"padding-left:10px; font-size: 11px\"> HW : ")
                    .append(mHwName)
                    .append(" - LOGO : ")
                    .append(sLogoFacing2String.get(mLogo))
                    .append(" - USB : ")
                    .append(sUsbFacing2String.get(mUsb))
                    .append("</p>");
        }
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
        if (mConfigurationValid) {
            result.append(header)
                    .append("> HW :")
                    .append(mHwName)
                    .append(" - LOGO : ")
                    .append(sLogoFacing2String.get(mLogo))
                    .append(" - USB : ")
                    .append(sUsbFacing2String.get(mUsb))
                    .append("\n");
        }
        return result.toString();
    }
}
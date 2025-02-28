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
import com.qualcomm.robotcore.hardware.HardwareMap;

/* ACME includes */
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;

/* FTC Controller */
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class ImuBuiltIn implements ImuComponent {

    public static final String sTypeKey    = "built-in";
    static final String sHwMapKey          = "hwmap";
    static final String sLogoDirectionKey  = "logo-direction";
    static final String sUsbDirectionKey   = "usb-direction";

    static final Map<String, RevHubOrientationOnRobot.LogoFacingDirection> sString2LogoFacing = Map.of(
            "up", RevHubOrientationOnRobot.LogoFacingDirection.UP,
            "down",RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
            "right",RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            "left",RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            "backward",RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
            "forward",RevHubOrientationOnRobot.LogoFacingDirection.FORWARD
    );
    static final Map<RevHubOrientationOnRobot.LogoFacingDirection, String> sLogoFacing2String = Map.of(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,      "up",
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN,    "down",
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,   "right",
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,    "left",
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,"backward",
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, "forward"
    );

    static final Map<String, RevHubOrientationOnRobot.UsbFacingDirection> sString2UsbFacing = Map.of(
            "up", RevHubOrientationOnRobot.UsbFacingDirection.UP,
            "down",RevHubOrientationOnRobot.UsbFacingDirection.DOWN,
            "right",RevHubOrientationOnRobot.UsbFacingDirection.RIGHT,
            "left",RevHubOrientationOnRobot.UsbFacingDirection.LEFT,
            "backward",RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD,
            "forward",RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    );
    static final Map<RevHubOrientationOnRobot.UsbFacingDirection, String> sUsbFacing2String = Map.of(
            RevHubOrientationOnRobot.UsbFacingDirection.UP,         "up",
            RevHubOrientationOnRobot.UsbFacingDirection.DOWN,       "down",
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT,      "right",
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT,       "left",
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD,   "backward",
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,"forward"
    );

    final LogManager                                mLogger;

    boolean                                         mConfigurationValid;
    final String                                    mName;
    String                                          mHwName;

    final HardwareMap                               mMap;
    LazyImu                                         mImu;
    RevHubOrientationOnRobot.LogoFacingDirection    mLogo;
    RevHubOrientationOnRobot.UsbFacingDirection     mUsb;

    AngularVelocity                                 mVelocity;
    YawPitchRollAngles                              mPosition;


    /* ----------------------- Constructors ------------------------ */
    /**
     * Creates an ImuComponent instance with a specified name, hardware map, and logger.
     *
     * @param name The unique identifier for the IMU component.
     * @param hwMap The FTC HardwareMap to retrieve the IMU hardware.
     * @param logger The logging manager to handle system logs.
     */
    public ImuBuiltIn(String name, HardwareMap hwMap, LogManager logger) {
        mConfigurationValid = false;
        mLogger             = logger;
        mName               = name;
        mHwName             = "";
        mMap                = hwMap;
        mImu                = null;
        mVelocity           = null;
        mPosition           = null;
    }


    /**
     * Retrieves the name of the IMU component.
     *
     * @return The name of the component.
     */
    public String                       name() { return mName; }

    /**
     * Logs the current imu yaw, pitch and roll.
     *
     * @return A formatted string containing imu telemetry data.
     */
    public String                       log() {
        String result = "";
        if(mConfigurationValid) {
            result += "\n Y : " + mPosition.getYaw(AngleUnit.DEGREES) + " P : " + mPosition.getPitch(AngleUnit.DEGREES) + " R : " + mPosition.getRoll(AngleUnit.DEGREES);
        }
        return result;
    }

    /**
     * Cache current imu value to enable multiple calls in a loop without
     */
    public void                         update()
    {
        if(mConfigurationValid) {
            mPosition = mImu.get().getRobotYawPitchRollAngles();
            mVelocity = mImu.get().getRobotAngularVelocity(AngleUnit.RADIANS);
        }
    }

    /**
     * Yaw reste function
     */
    public void                         reset() {
        mImu.get().resetYaw();
    }

    /**
     * Retrieves the current heading (yaw angle) of the robot in radians.
     *
     * @return The heading in radians, adjusted with the configured offset.
     */
    public double                       heading() {
        double result = 0;
        if(mConfigurationValid && mPosition != null) {
            result = mPosition.getYaw(AngleUnit.RADIANS);
        }
        return result;
    }

    /**
     * Retrieves the current angular velocity around the Z-axis in radians per second.
     *
     * @return The IMU's angular velocity in radians per second.
     */
    public double                       headingVelocity() {
        double result = 0;
        if(mConfigurationValid && mVelocity != null) {
            result = mVelocity.zRotationRate;
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
        mConfigurationValid = true;
        mImu = null;
        mHwName = "";

        try {
            if(reader.has(sHwMapKey)) {
                mHwName = reader.getString(sHwMapKey);
            }

            RevHubOrientationOnRobot orientation = null;
            if(reader.has(sLogoDirectionKey) && reader.has(sUsbDirectionKey)) {
                if(sString2LogoFacing.containsKey(reader.getString(sLogoDirectionKey))) {
                    mLogo = sString2LogoFacing.get(reader.getString(sLogoDirectionKey));
                }
                if(sString2UsbFacing.containsKey(reader.getString(sUsbDirectionKey))) {
                    mUsb = sString2UsbFacing.get(reader.getString(sUsbDirectionKey));
                }

                orientation = new RevHubOrientationOnRobot(mLogo, mUsb);
            }

            if(mMap != null && orientation != null && !mHwName.isEmpty())
            {
                mImu = new LazyHardwareMapImu(mMap, mHwName, orientation);
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
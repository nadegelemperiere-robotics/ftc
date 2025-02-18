/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * Axis class provides an abstraction for accessing
 * gamepad analog axes (e.g., triggers or joystick axes)
 * using reflection.
 * -------------------------------------------------------
 * <p>
 * This class allows configurable input scaling, deadzone
 * handling, and maximum value capping for gamepad axes.
 * It reads values dynamically via reflection, making it
 * adaptable to different hardware configurations.
 * <p>
 * Features:
 * - Reads gamepad axis values dynamically.
 * - Supports deadzone and maximum value configurations.
 * - Multiplies input values for sensitivity adjustments.
 * - Logs configuration in text and HTML formats.
 * - Stores and retrieves configurations using JSON.
 * <p>
 * Dependencies:
 * - Qualcomm Robotics SDK (Gamepad API)
 * - JSON Processing (org.json)
 * - Custom LogManager for logging
 * <p>
 * Usage:
 * 1. Create an `Axis` instance, linking it to a specific
 *    gamepad axis.
 * 2. Optionally configure deadzones and scaling.
 * 3. Read values using `value()`.
 * <p>
 * Example:
 * {@code
 *      Axis trigger = new Axis(gamepad1, "left_trigger", 1.0, logger);
 *      double triggerValue = trigger.value();
 * }
 */


package org.firstinspires.ftc.core.components.controllers;

/* System includes */
import java.lang.reflect.Field;

/* JSON includes */
import org.json.JSONObject;
import org.json.JSONException;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Gamepad;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

public class Axis implements Configurable {

    public  static  final   double      sDefaultMaximum = 1.0;
    public  static  final   double      sDefaultDeadzone = 0.0;

            static  final   String      sDeadzoneKey   = "deadzone";
            static  final   String      sMaximumKey    = "maximum";

                    final   LogManager  mLogger;

                            boolean     mConfigurationValid;

                    final   Gamepad     mGamepad;
                    final   String      mName;

                    final   double      mMultiplier;
                            double      mDeadZone;
                            double      mMaximum;

    /**
     * Axis constructor
     *
     * @param gamepad qualcomm controller to extend
     * @param name qualcomm button member name for reflected access
     * @param logger logger
     */
    public Axis(Gamepad gamepad, String name, LogManager logger) {
        mLogger     = logger;
        mGamepad    = gamepad;
        mName       = name;
        mMultiplier = 1.0;
        mMaximum    = sDefaultMaximum;
        mDeadZone   = sDefaultDeadzone;
    }

    /**
     * Axis constructor
     *
     * @param gamepad qualcomm controller to extend
     * @param name qualcomm button member name for reflected access
     * @param multiplier value multiplier
     * @param logger logger
     */
    public Axis(Gamepad gamepad, String name, double multiplier, LogManager logger) {
        mLogger     = logger;
        mGamepad    = gamepad;
        mName       = name;
        mMultiplier = multiplier;
        mMaximum    = sDefaultMaximum;
        mDeadZone   = sDefaultDeadzone;
    }

    /**
     * Deadzone accessor
     *
     * @param deadzone : value under which trigger value should be considered null
     */
    public  void    deadzone(double deadzone) {
        if (deadzone >= 0 && deadzone <= 1.0) { mDeadZone = deadzone; }
    }

    /**
     * Maximum accessor
     *
     * @param maximum : maximum value of the trigger
     */
    public  void    maximum(double maximum) {
        if (maximum >= 0 && maximum <= 1.0) { mMaximum = maximum; }
    }

    /**
     * Value accessor
     *
     * @return trigger value
     */
    public double   value() {

        double result = 0;

        if(mGamepad != null) {

            try {

                Field field = Gamepad.class.getDeclaredField(mName);
                Object status = field.get(mGamepad);
                if (status != null) {
                    if (field.getType() == double.class) {
                        result = ((double) status * mMultiplier);
                    } else if (field.getType() == float.class) {
                        result = ((float) status * mMultiplier);
                    }
                }

                result = Axis.applyDeadzone(result, mDeadZone, mMaximum);
            }
            catch(NoSuchFieldException | NullPointerException | IllegalAccessException e ) {
                mLogger.error(e.getMessage());
            }

        }
        
        return result;
    }


    /**
     * Configuration checking
     *
     * @return true if object is correctly configured, false otherwise
     */
    public boolean  isConfigured() {
        return mConfigurationValid;
    }

    /**
     * Configuration logging into HTML
     *
     * @return configuration as html string
     */
    public String   logConfigurationHTML() {

        String result = "<li style=\"padding-left:10px;font-size:" +
                LogManager.sMetricFontSize +
                "px\"> " +
                sDeadzoneKey +
                " : " +
                mDeadZone +
                "</li>" +
                "<li style=\"padding-left:10px;font-size:" +
                LogManager.sMetricFontSize +
                "px\"> " +
                sMaximumKey +
                " : " +
                mMaximum +
                "</li>";

        return result;
    }
    /**
     * Configuration logging into text
     *
     * @return configuration as basic string
     */
    public String  logConfigurationText(String header) {

        String result = header +
                sDeadzoneKey +
                " : " +
                mDeadZone +
                ", " +
                sMaximumKey +
                " : " +
                mMaximum;

        return result;
    }

    /**
     * Reads log manager configuration
     *
     * @param reader : JSON object containing configuration
     */
    public void read(JSONObject reader) {

        mConfigurationValid = true;

        if(reader.has(sDeadzoneKey)) {
            try {
                double deadzone = reader.getDouble(sDeadzoneKey);
                this.deadzone(deadzone);
            }
            catch(JSONException e) {
                mLogger.error("Error in controller configuration");
                mConfigurationValid = false;
            }
        }

        if(reader.has(sMaximumKey)) {
            try {
                double maximum = reader.getDouble(sMaximumKey);
                this.maximum(maximum);
            }
            catch(JSONException e) {
                mLogger.error("Error in controller configuration");
                mConfigurationValid = false;
            }
        }
    }

    /**
     * Writes log manager configuration
     *
     * @param writer : JSON object to store configuration
     */
    public void write(JSONObject writer) {

        try {
            if(mConfigurationValid) {
                writer.put(sDeadzoneKey,mDeadZone);
                writer.put(sMaximumKey,mMaximum);
            }
        }
        catch(JSONException e ) { mLogger.error("Error writing configuration"); }
    }

    /**
     * Deadzone scaling function
     *
     * @param value : raw trigger value
     * @param deadzone : deadzone value
     * @param maximum : maximum allowed value
     */
    private static double applyDeadzone(double value, double deadzone, double maximum) {
        if (Math.abs(value) < deadzone) {
            return 0.0; // Inside deadzone
        }
        // Scale the value to account for the deadzone
        return ((value - Math.signum(value) * deadzone) / (1.0 - deadzone) * maximum);
    }

}

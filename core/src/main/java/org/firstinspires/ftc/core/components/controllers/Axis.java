/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   SmartButton class that manages gamepad button
   --> Adds the capability to detect when button is pressed
   and was released before, to trigger events only once on button
   pressed
   --> Adds the capability to use triggers has buttons with a
   threshold
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.controllers;

/* System includes */
import java.lang.reflect.Field;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Gamepad;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class Axis {

    LogManager  mLogger;

    Gamepad     mGamepad;
    String      mName;

    double      mMultiplier;
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
        mMaximum    = 1.0;
        mDeadZone   = 0.0;
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
        mMaximum    = 1.0;
        mDeadZone   = 0.0;
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
    public double value() {

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

    private static double applyDeadzone(double value, double deadzone, double maximum) {
        if (Math.abs(value) < deadzone) {
            return 0.0; // Inside deadzone
        }
        // Scale the value to account for the deadzone
        return ((value - Math.signum(value) * deadzone) / (1.0 - deadzone) * maximum);
    }

}

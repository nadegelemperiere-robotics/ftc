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

public class Button {

    static  final   double      sTriggerThreshold = 0;

                    LogManager  mLogger;

                    Gamepad     mGamepad;
                    String      mName;


                    boolean     mWasPressed;
                    boolean     mWasReleased;
                    double      mMultiplier;

    /**
     * Button constructor
     *
     * @param gamepad qualcomm controller to extend
     * @param name qualcomm button member name for reflected access
     * @param logger logger
     */
    public Button(Gamepad gamepad, String name, LogManager logger) {
        mLogger         = logger;
        mGamepad        = gamepad;
        mName           = name;
        mWasPressed     = false;
        mWasReleased    = true;
        mMultiplier     = 1.0;
    }

    /**
     * Button constructor
     *
     * @param gamepad qualcomm controller to extend
     * @param name qualcomm button member name for reflected access
     * @param multiplier trigger value multiplier to be considered as a button (positive value)
     * @param logger logger
     */
    public Button(Gamepad gamepad, String name, double multiplier, LogManager logger) {
        mLogger         = logger;
        mGamepad        = gamepad;
        mName           = name;
        mWasPressed     = false;
        mWasReleased    = true;
        mMultiplier     = multiplier;
    }

    /**
     * Check if button is being pressed
     *
     * @return true if button is being pressed
     */
    public boolean pressed() {

        boolean result = false;

        if(mGamepad != null) {

            try {

                Field field = Gamepad.class.getDeclaredField(mName);
                Object status = field.get(mGamepad);
                if (status != null) {
                    if (field.getType() == boolean.class) {
                        result = (boolean) status;
                    } else if (field.getType() == double.class) {
                        result = ((double) status * mMultiplier > sTriggerThreshold);
                    } else if (field.getType() == float.class) {
                        result = ((float) status * mMultiplier > sTriggerThreshold);
                    }
                }
            }
            catch(NoSuchFieldException | NullPointerException | IllegalAccessException e ) {
                mLogger.error(e.getMessage());
            }

        }

        if(result) { mLogger.trace("Button " + mName + " is pressed"); }
        
        return result;
    }

    /**
     * Check if button is not being pressed
     *
     * @return true if button is not being pressed
     */
    public boolean notPressed() {
        return !this.pressed();
    }

    /**
     * Check if button has just been pressed
     *
     * @return true if the button has just been pressed. False if not pressed or keep being pressed
     */
    public boolean pressedOnce() {

        boolean is_pressed = this.pressed();
        boolean result = is_pressed && mWasReleased;
        mWasReleased = !is_pressed;

        if(result) { mLogger.trace("Button " + mName + " is first pressed"); }
        
        return result;
    }

    /**
     * Check if button has just been released
     *
     * @return true if the button has just been released. False if pressed or keep being released
     */
    public boolean releasedOnce() {

        boolean is_pressed = this.pressed();
        boolean result = !is_pressed && mWasPressed;
        mWasPressed = is_pressed;

        if(result) { mLogger.trace("Button " + mName + " is released"); }

        return result;
    }
}

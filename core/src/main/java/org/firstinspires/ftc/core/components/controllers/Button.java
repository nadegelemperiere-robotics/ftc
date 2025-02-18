/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * Button class provides an abstraction for handling
 * gamepad buttons using reflection.
 * -------------------------------------------------------
 * <p>
 * This class allows checking button states, including:
 * - Whether a button is currently pressed.
 * - Whether a button was pressed once.
 * - Whether a button was released once.
 * <p>
 * Features:
 * - Uses reflection to access button fields dynamically.
 * - Supports trigger-based buttons with threshold detection.
 * - Implements edge detection for press and release events.
 * - Provides logging capabilities for button actions.
 * <p>
 * Dependencies:
 * - Qualcomm Robotics SDK (Gamepad API)
 * - Custom LogManager for logging
 * <p>
 * Usage:
 * 1. Create a `Button` instance, linking it to a gamepad button.
 * 2. Call `pressed()`, `pressedOnce()`, or `releasedOnce()`
 *    to detect button states.
 * <p>
 * Example:
 * {@code
 *      Button aButton = new Button(gamepad1, "a", logger);
 *      if (aButton.pressedOnce()) {
 *          // Execute action when button "A" is first pressed
 *      }
 * }
 */


package org.firstinspires.ftc.core.components.controllers;

/* System includes */
import java.lang.reflect.Field;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Gamepad;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class Button {

    static  final   double      sTriggerThreshold = 0;

            final   LogManager  mLogger;

            final   Gamepad     mGamepad;
            final   String      mName;


                    boolean     mWasPressed;
                    boolean     mWasReleased;
            final   double      mMultiplier;

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

        boolean result = !(this.pressed());
        if(result) { mLogger.trace("Button " + mName + " is not pressed"); }
        return result;
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

/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * Controller class provides an abstraction layer for
 * managing gamepad inputs, including buttons and axes.
 * -------------------------------------------------------
 * <p>
 * This class extends the standard FTC Gamepad by:
 * - Providing easy access to buttons and axes.
 * - Implementing press and release detection for buttons.
 * - Supporting scaling and deadzone adjustments for axes.
 * - Allowing configuration storage via JSON.
 * - Logging configuration details in both HTML and text formats.
 * <p>
 * Features:
 * - Supports all FTC Gamepad buttons and axes.
 * - Provides structured access via `buttons` and `axes` inner classes.
 * - Configurable deadzone and scaling for analog inputs.
 * - Edge detection for button press/release events.
 * - Reads and writes configurations in JSON format.
 * - Supports logging for debugging.
 * <p>
 * Dependencies:
 * - Qualcomm Robotics SDK (Gamepad API)
 * - JSON Processing (org.json)
 * - Custom LogManager for logging
 * <p>
 * Usage:
 * 1. Create a `Controller` instance linked to a `Gamepad`.
 * 2. Access button states via `buttons.a.pressed()` etc.
 * 3. Read joystick/trigger values via `axes.left_stick_x.value()`.
 * 4. Store and retrieve configurations in JSON format.
 * <p>
 * Example:
 * {@code
 *      Controller controller = new Controller(gamepad1, logger);
 * <p>
 *      if (controller.buttons.a.pressedOnce()) {
 *          // Action when button A is first pressed
 *      }
 * <p>
 *      double joystickX = controller.axes.left_stick_x.value();
 * }
 */

package org.firstinspires.ftc.core.components.controllers;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Gamepad;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;


public class Controller implements Configurable {

    // Json keys
    static  final   String  sAxesKey       = "axes";


    public  static  class     Buttons {
        public Button a;
        public Button b;
        public Button x;
        public Button y;

        public Button dpad_up;
        public Button dpad_down;
        public Button dpad_left;
        public Button dpad_right;

        public Button left_bumper;
        public Button left_trigger;

        public Button left_stick_x_right;
        public Button left_stick_x_left;
        public Button left_stick_y_up;
        public Button left_stick_y_down;
        public Button left_stick_button;

        public Button right_bumper;
        public Button right_trigger;

        public Button right_stick_x_right;
        public Button right_stick_x_left;
        public Button right_stick_y_up;
        public Button right_stick_y_down;
        public Button right_stick_button;
    }

    public static class Axes {

        public Axis left_stick_x;
        public Axis left_stick_y;
        public Axis left_trigger;
        public Axis right_stick_x;
        public Axis right_stick_y;
        public Axis right_trigger;

    }

    public  final   Buttons     buttons;
    public  final   Axes        axes;

            final   LogManager  mLogger;

            boolean     mConfigurationValid;

    /**
     * Controller constructor
     *
     * @param gamepad qualcomm controller to extend
     * @param logger logger
     */
    public              Controller(Gamepad gamepad, LogManager logger) {

        mLogger = logger;
        mConfigurationValid = true;

        buttons = new Buttons();

        buttons.a = new Button(gamepad, "a", logger);
        buttons.b = new Button(gamepad, "b", logger);
        buttons.x = new Button(gamepad, "x", logger);
        buttons.y = new Button(gamepad, "y", logger);

        buttons.dpad_up = new Button(gamepad, "dpad_up", logger);
        buttons.dpad_down = new Button(gamepad, "dpad_down", logger);
        buttons.dpad_left = new Button(gamepad, "dpad_left", logger);
        buttons.dpad_right = new Button(gamepad, "dpad_right", logger);

        buttons.left_bumper        = new Button(gamepad, "left_bumper", logger);
        buttons.left_trigger       = new Button(gamepad, "left_trigger", logger);
        buttons.left_stick_x_left  = new Button(gamepad, "left_stick_x", -1.0, logger );
        buttons.left_stick_x_right = new Button(gamepad, "left_stick_x", logger);
        buttons.left_stick_y_up    = new Button(gamepad, "left_stick_y", -1.0, logger);
        buttons.left_stick_y_down  = new Button(gamepad, "left_stick_y", logger);
        buttons.left_stick_button  = new Button(gamepad, "left_stick_button", logger);

        buttons.right_bumper        = new Button(gamepad, "right_bumper", logger);
        buttons.right_trigger       = new Button(gamepad, "right_trigger", logger);
        buttons.right_stick_x_left  = new Button(gamepad, "right_stick_x", -1.0, logger);
        buttons.right_stick_x_right = new Button(gamepad, "right_stick_x", logger);
        buttons.right_stick_y_up    = new Button(gamepad, "right_stick_y", -1.0,logger);
        buttons.right_stick_y_down  = new Button(gamepad, "right_stick_y", logger);
        buttons.right_stick_button  = new Button(gamepad, "right_stick_button", logger);

        axes = new Axes();

        axes.left_stick_x  = new Axis(gamepad, "left_stick_x", logger);
        axes.left_stick_y  = new Axis(gamepad, "left_stick_y", -1.0, logger);
        axes.left_trigger  = new Axis(gamepad, "left_trigger", logger);
        axes.right_stick_x = new Axis(gamepad, "right_stick_x", logger);
        axes.right_stick_y = new Axis(gamepad, "right_stick_y", -1.0, logger);
        axes.right_trigger = new Axis(gamepad, "right_trigger", logger);

    }

    /**
     * Configuration checking
     *
     * @return true if object is correctly configured, false otherwise
     */
    public boolean isConfigured() {
        return mConfigurationValid;
    }

    /**
     * Configuration logging into HTML
     *
     * @return configuration as html string
     */
    public String  logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        result.append("<details style=\"margin-left:10px\">\n")
                .append("<summary style=\"font-size: 12px; font-weight: 500\"> AXES </summary>\n")
                .append("<ul>\n");

        result.append("<li>\n")
                .append("<details style=\"margin-left:10px\">\n")
                .append("<summary style=\"font-size: 12px; font-weight: 500\"> LEFT STICK X </summary>\n")
                .append("<ul>\n")
                .append(this.axes.left_stick_x.logConfigurationHTML())
                .append("</ul>\n")
                .append("</details>\n")
                .append("</li>\n");

        result.append("<li>\n")
                .append("<details style=\"margin-left:10px\">\n")
                .append("<summary style=\"font-size: 12px; font-weight: 500\"> LEFT STICK Y </summary>\n")
                .append("<ul>\n")
                .append(this.axes.left_stick_y.logConfigurationHTML())
                .append("</ul>\n")
                .append("</details>\n")
                .append("</li>\n");

        result.append("<li>\n")
                .append("<details style=\"margin-left:10px\">\n")
                .append("<summary style=\"font-size: 12px; font-weight: 500\"> LEFT TRIGGER </summary>\n")
                .append("<ul>\n")
                .append(this.axes.left_trigger.logConfigurationHTML())
                .append("</ul>\n")
                .append("</details>\n")
                .append("</li>\n");

        result.append("<li>\n")
                .append("<details style=\"margin-left:10px\">\n")
                .append("<summary style=\"font-size: 12px; font-weight: 500\"> RIGHT STICK X </summary>\n")
                .append("<ul>\n")
                .append(this.axes.right_stick_x.logConfigurationHTML())
                .append("</ul>\n")
                .append("</details>\n")
                .append("</li>\n");

        result.append("<li>\n")
                .append("<details style=\"margin-left:10px\">\n")
                .append("<summary style=\"font-size: 12px; font-weight: 500\"> RIGHT STICK Y </summary>\n")
                .append("<ul>\n")
                .append(this.axes.right_stick_y.logConfigurationHTML())
                .append("</ul>\n")
                .append("</details>\n")
                .append("</li>\n");

        result.append("<li>\n")
                .append("<details style=\"margin-left:10px\">\n")
                .append("<summary style=\"font-size: 12px; font-weight: 500\"> RIGHT TRIGGER </summary>\n")
                .append("<ul>\n")
                .append(this.axes.right_trigger.logConfigurationHTML())
                .append("</ul>\n")
                .append("</details>\n")
                .append("</li>\n");

        result.append("</ul>\n")
                .append("</details>\n");

        return result.toString();
    }
    /**
     * Configuration logging into HTML
     *
     * @return configuration as html string
     */
    public String  logConfigurationText(String header) {

        String result = header +
                "> axes :\n" +
                header +
                "--> left_stick_x : " +
                this.axes.left_stick_x.logConfigurationText("") +
                "\n" +
                header +
                "--> left_stick_y : " +
                this.axes.left_stick_y.logConfigurationText("") +
                "\n" +
                header +
                "--> left_trigger : " +
                this.axes.left_trigger.logConfigurationText("") +
                "\n" +
                header +
                "--> right_stick_x : " +
                this.axes.right_stick_x.logConfigurationText("") +
                "\n" +
                header +
                "--> right_stick_y : " +
                this.axes.right_stick_y.logConfigurationText("") +
                "\n" +
                header +
                "--> right_trigger : " +
                this.axes.right_trigger.logConfigurationText("") +
                "\n";

            return result;

    }

    /**
     * Reads boolean logic configuration
     *
     * @param reader : JSON object containing configuration
     */
    public void read(JSONObject reader) {

        mConfigurationValid = true;

        if( reader.has(sAxesKey)) {
            try {
                JSONObject axes = reader.getJSONObject(sAxesKey);
                if (axes.has("left_stick_x")) {
                    JSONObject axis = axes.getJSONObject("left_stick_x");
                    this.axes.left_stick_x.read(axis);
                    if(!this.axes.left_stick_x.isConfigured()) { mConfigurationValid = false; }
                }
                if (axes.has("left_stick_y")) {
                    JSONObject axis = axes.getJSONObject("left_stick_y");
                    this.axes.left_stick_y.read(axis);
                    if(!this.axes.left_stick_y.isConfigured()) { mConfigurationValid = false; }
                }
                if (axes.has("left_trigger")) {
                    JSONObject axis = axes.getJSONObject("left_trigger");
                    this.axes.left_trigger.read(axis);
                    if(!this.axes.left_trigger.isConfigured()) { mConfigurationValid = false; }
                }
                if (axes.has("right_stick_x")) {
                    JSONObject axis = axes.getJSONObject("right_stick_x");
                    this.axes.right_stick_x.read(axis);
                    if(!this.axes.right_stick_x.isConfigured()) { mConfigurationValid = false; }
                }
                if (axes.has("right_stick_y")) {
                    JSONObject axis = axes.getJSONObject("right_stick_y");
                    this.axes.right_stick_y.read(axis);
                    if(!this.axes.right_stick_y.isConfigured()) { mConfigurationValid = false; }
                }
                if (axes.has("right_trigger")) {
                    JSONObject axis = axes.getJSONObject("right_trigger");
                    this.axes.right_trigger.read(axis);
                    if(!this.axes.right_trigger.isConfigured()) { mConfigurationValid = false; }
                }
            }
            catch(JSONException e ) { mLogger.error("Error reading configuration"); }
        }

    }

    /**
     * Writes log manager configuration
     *
     * @param writer : JSON object to store configuration
     */
    public void write(JSONObject writer) {

        if(mConfigurationValid) {
            JSONObject axes = new JSONObject();

            try {

                JSONObject leftStickX = new JSONObject();
                this.axes.left_stick_x.write(leftStickX);
                axes.put("left_stick_x", leftStickX);

                JSONObject leftStickY = new JSONObject();
                this.axes.left_stick_y.write(leftStickY);
                axes.put("left_stick_y", leftStickY);

                JSONObject leftTrigger = new JSONObject();
                this.axes.left_trigger.write(leftTrigger);
                axes.put("left_trigger", leftTrigger);

                JSONObject rightStickX = new JSONObject();
                this.axes.right_stick_x.write(rightStickX);
                axes.put("right_stick_x", rightStickX);

                JSONObject rightStickY = new JSONObject();
                this.axes.right_stick_y.write(rightStickY);
                axes.put("right_stick_y", rightStickY);

                JSONObject rightTrigger = new JSONObject();
                this.axes.right_trigger.write(rightTrigger);
                axes.put("right_trigger", rightTrigger);

                writer.put(sAxesKey, axes);
            }
            catch(JSONException e ) { mLogger.error("Error writing configuration"); }
        }
    }


}


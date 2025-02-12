/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Gamepad extended management
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.controllers;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Gamepad;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class Controller {

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

    public  Buttons     buttons;
    public  Axes        axes;

            LogManager  mLogger;

    /**
     * Controller constructor
     *
     * @param gamepad qualcomm controller to extend
     * @param logger logger
     */
    public              Controller(Gamepad gamepad, LogManager logger) {

        mLogger = logger;

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

}


/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Controller test class
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.test.components;

/* System includes */
import java.io.File;

/* Android includes */
import android.os.Environment;

/* Junit 5 includes */
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/* Mockito includes */
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.mockito.junit.jupiter.MockitoExtension;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Gamepad;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Component Under Test includes */
import org.firstinspires.ftc.core.components.controllers.Controller;

@ExtendWith(MockitoExtension.class)
public class ControllerTest {

    private LogManager      mLogger;
    private Controller      mController;
    private Gamepad         mGamepad;
    private Configuration   mConfiguration;

    @BeforeEach
    public void setUp() {
        try (MockedStatic<Environment> mockedEnvironment = Mockito.mockStatic(Environment.class)) {
            mockedEnvironment.when(Environment::getExternalStorageDirectory).thenReturn(new File(getClass().getClassLoader().getResource("results").getFile()));
            mLogger = new LogManager(null, null, "controller-test",2);
            mLogger.level(LogManager.Severity.INFO);
            mLogger.info("Setting it up!");
        }
    }

    @AfterEach
    public void tearDown() {
        mLogger.stop();
    }

    @Test
    public void button() {

        mGamepad = new Gamepad();
        mController = new Controller(mGamepad, mLogger);

        // Test A button
        assertFalse(mController.buttons.a.pressed(),       "A is initialized as pressed");
        assertFalse(mController.buttons.a.pressedOnce(),   "A is initialized as pressed");
        assertFalse(mController.buttons.a.releasedOnce(),  "A is initialized as released");
        mGamepad.a = true;
        assertTrue(mController.buttons.a.pressed(),        "A should be pressed");
        assertTrue(mController.buttons.a.pressedOnce(),    "A should be pressed for the first time");
        assertFalse(mController.buttons.a.releasedOnce(),  "A should not be released");
        assertTrue(mController.buttons.a.pressed(),        "A should be pressed");
        assertFalse(mController.buttons.a.pressedOnce(),   "A should no longer be pressed for the first time");
        assertFalse(mController.buttons.a.releasedOnce(),  "A should not be released");
        mGamepad.a = false;
        assertFalse(mController.buttons.a.pressed(),      "A should not be pressed");
        assertFalse(mController.buttons.a.pressedOnce(),  "A should no longer be pressed for the first time");
        assertTrue(mController.buttons.a.releasedOnce(),  "A should be released for the first time");
        assertFalse(mController.buttons.a.pressed(),      "A should not be pressed");
        assertFalse(mController.buttons.a.pressedOnce(),  "A should not be pressed");
        assertFalse(mController.buttons.a.releasedOnce(), "A should no longer be released for the first time");

        // Test left stick

        assertFalse(mController.buttons.left_stick_y_up.pressed(),      "Left stick is initialized as pressed");
        assertFalse(mController.buttons.left_stick_y_up.pressedOnce(),  "Left stick is initialized as pressed");
        assertFalse(mController.buttons.left_stick_y_up.releasedOnce(), "Left stick is initialized as released");
        mGamepad.left_stick_y = -0.2f;
        assertTrue(mController.buttons.left_stick_y_up.pressed(),       "Left stick should be pressed");
        assertTrue(mController.buttons.left_stick_y_up.pressedOnce(),   "Left stick should be pressed for the first time");
        assertFalse(mController.buttons.left_stick_y_up.releasedOnce(), "Left stick should not be released");
        assertTrue(mController.buttons.left_stick_y_up.pressed(),       "Left stick should be pressed");
        assertFalse(mController.buttons.left_stick_y_up.pressedOnce(),  "Left stick should no longer be pressed for the first time");
        assertFalse(mController.buttons.left_stick_y_up.releasedOnce(), "Left stick should not be released");
        mGamepad.left_stick_y = 0.0f;
        assertFalse(mController.buttons.left_stick_y_up.pressed(),      "Left stick should not be pressed");
        assertFalse(mController.buttons.left_stick_y_up.pressedOnce(),  "Left stick should no longer be pressed for the first time");
        assertTrue(mController.buttons.left_stick_y_up.releasedOnce(),  "Left stick should be released for the first time");
        assertFalse(mController.buttons.left_stick_y_up.pressed(),      "Left stick should not be pressed");
        assertFalse(mController.buttons.left_stick_y_up.pressedOnce(),  "Left stick should not be pressed");
        assertFalse(mController.buttons.left_stick_y_up.releasedOnce(), "Left stick should no longer be released for the first time");

    }

    @Test
    public void axis() {

        mGamepad = new Gamepad();
        mController = new Controller(mGamepad, mLogger);

        // Test right stick
        mGamepad.right_stick_y = -0.2f;
        assertEquals(0.2, mController.axes.right_stick_y.value(), 0.0000001,  "Y axis should be inverted");
        mGamepad.right_stick_y = 0.2f;
        assertEquals(-0.2, mController.axes.right_stick_y.value(), 0.0000001, "Y axis should be inverted");
        mGamepad.right_stick_x = 0.2f;
        assertEquals(0.2, mController.axes.right_stick_x.value(),  0.0000001, "X axis should not be inverted");
        mGamepad.right_stick_x = -0.2f;
        assertEquals(-0.2, mController.axes.right_stick_x.value(), 0.0000001, "X axis should not be inverted");

        // Test left stick
        mGamepad.left_stick_y = -0.2f;
        assertEquals(0.2, mController.axes.left_stick_y.value(),  0.0000001,"Y axis should be inverted");
        mGamepad.left_stick_y = 0.2f;
        assertEquals(-0.2, mController.axes.left_stick_y.value(), 0.0000001, "Y axis should be inverted");
        mGamepad.left_stick_x = 0.2f;
        assertEquals(0.2, mController.axes.left_stick_x.value(),  0.0000001, "X axis should not be inverted");
        mGamepad.left_stick_x = -0.2f;
        assertEquals(-0.2, mController.axes.left_stick_x.value(), 0.0000001, "X axis should not be inverted");
    }

    @Test
    public void read() {

        mConfiguration = new Configuration(mLogger);
        mGamepad = new Gamepad();
        mController = new Controller(mGamepad, mLogger);
        mConfiguration.register("controller", mController);
        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/controller-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is valid");

        mGamepad.left_stick_x = -0.1f;
        assertEquals(0, mController.axes.left_stick_x.value(), 0.0000001, "X axis deadzone should be applied");
        mGamepad.left_stick_x = 0.1f;
        assertEquals(0, mController.axes.left_stick_x.value(), 0.0000001, "X axis deadzone should be applied");

        mGamepad.left_stick_x = -0.25f;
        assertEquals(-0.05, mController.axes.left_stick_x.value(), 0.0000001, "X axis deadzone scaling should be applied");
        mGamepad.left_stick_x = 0.25f;
        assertEquals(0.05, mController.axes.left_stick_x.value(), 0.0000001, "X axis deadzone scaling should be applied");

        mGamepad.left_stick_x = -1.0f;
        assertEquals(-0.8, mController.axes.left_stick_x.value(), 0.0000001, "X axis deadzone scaling should be applied");
        mGamepad.left_stick_x = 1.0f;
        assertEquals(0.8, mController.axes.left_stick_x.value(), 0.0000001, "X axis deadzone scaling should be applied");

        mGamepad.left_stick_y = -0.05f;
        assertEquals(0, mController.axes.left_stick_y.value(), 0.0000001, "Y axis deadzone should be applied");
        mGamepad.left_stick_y = 0.05f;
        assertEquals(0, mController.axes.left_stick_y.value(), 0.0000001, "Y axis deadzone should be applied");

        mGamepad.left_stick_y = -0.25f;
        assertEquals(0.15, mController.axes.left_stick_y.value(), 0.0000001, "Y axis deadzone scaling should be applied");
        mGamepad.left_stick_y = 0.25f;
        assertEquals(-0.15, mController.axes.left_stick_y.value(), 0.0000001, "Y axis deadzone scaling should be applied");

        mGamepad.left_stick_y = -1.0f;
        assertEquals(0.9, mController.axes.left_stick_y.value(), 0.0000001, "Y axis deadzone scaling should be applied");
        mGamepad.left_stick_y = 1.0f;
        assertEquals(-0.9, mController.axes.left_stick_y.value(), 0.0000001, "Y axis deadzone scaling should be applied");


    }

    @Test
    public void write() {

        mConfiguration = new Configuration(mLogger);
        mGamepad = new Gamepad();
        mController = new Controller(mGamepad, mLogger);
        mConfiguration.register("controller", mController);
        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/controller-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is valid");

        mConfiguration.log();
        mLogger.update();

        mConfiguration.write(getClass().getClassLoader().getResource("results").getFile() + "/controller-write.json");

    }


}

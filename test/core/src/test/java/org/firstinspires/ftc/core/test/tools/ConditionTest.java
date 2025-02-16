/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Configuration management test class
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.test.tools;

/* System includes */
import java.io.File;

/* Android includes */
import android.os.Environment;

/* Junit 5 includes */
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/* Mockito includes */
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.mockito.junit.jupiter.MockitoExtension;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Gamepad;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Component includes */
import org.firstinspires.ftc.core.components.controllers.Controller;

/* Component Under Test includes */
import org.firstinspires.ftc.core.tools.Condition;

@ExtendWith(MockitoExtension.class)
public class ConditionTest {

    private LogManager      mLogger;
    private Controller      mController;
    private Gamepad         mGamepad;
    private Condition       mCondition;

    @BeforeEach
    public void setUp() {
        try (MockedStatic<Environment> mockedEnvironment = Mockito.mockStatic(Environment.class)) {
            mockedEnvironment.when(Environment::getExternalStorageDirectory).thenReturn(new File(getClass().getClassLoader().getResource("results").getFile()));
            mLogger = new LogManager(null, null, "condition-test",3);
            mLogger.level(LogManager.Severity.INFO);
            mLogger.info("Setting it up!");
        }
    }

    @Test
    public void evaluateStatic() {

        mGamepad        = new Gamepad();
        mController     = new Controller(mGamepad, mLogger);

        mGamepad.a = false;
        mGamepad.b = true;
        mGamepad.dpad_up = false;
        mCondition =  Condition.and(
                Condition.or(
                        new Condition(() -> mController.buttons.a.pressed()),
                        new Condition(() -> mController.buttons.b.pressed())),
                Condition.not(
                        new Condition(() -> mController.buttons.dpad_up.pressed())));

        assertTrue(mCondition.evaluate(), "Condition should be true");

        mGamepad.a = true;
        mGamepad.b = false;
        mGamepad.dpad_up = true;
        mCondition =  Condition.and(
                Condition.or(
                        new Condition(() -> mController.buttons.a.pressed()),
                        new Condition(() -> mController.buttons.b.pressed())),
                Condition.not(
                        new Condition(() -> mController.buttons.dpad_up.pressed())));

        assertFalse(mCondition.evaluate(), "Condition should be false");

    }

    @Test
    public void evaluateDynamic() {

        mGamepad        = new Gamepad();
        mController     = new Controller(mGamepad, mLogger);

        mCondition =  Condition.and(
                Condition.or(
                        new Condition(() -> mController.buttons.a.pressed()),
                        new Condition(() -> mController.buttons.b.pressedOnce())),
                Condition.not(
                        new Condition(() -> mController.buttons.dpad_up.releasedOnce())));

        mGamepad.a = false;
        mGamepad.b = true;
        mGamepad.dpad_up = false;

        assertTrue(mCondition.evaluate(), "Condition should be true");

        mGamepad.a = true;
        mGamepad.b = false;
        mGamepad.dpad_up = true;

        assertTrue(mCondition.evaluate(), "Condition should be true");

        mGamepad.a = true;
        mGamepad.b = true;
        mGamepad.dpad_up = false;

        assertFalse(mCondition.evaluate(), "Condition should be false");


    }

}



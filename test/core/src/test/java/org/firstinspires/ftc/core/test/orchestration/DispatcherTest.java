/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Scheduler test class
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.test.orchestration;

/* System includes */
import java.io.File;
import java.util.Map;
import java.util.LinkedHashMap;

/* Android includes */
import android.os.Environment;

/* Junit 5 includes */
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
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
import org.firstinspires.ftc.core.tools.Condition;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Component includes */
import org.firstinspires.ftc.core.components.controllers.Controller;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Robot;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.dispatcher.Dispatcher;

@ExtendWith(MockitoExtension.class)
public class DispatcherTest {

    private LogManager      mLogger;
    private Configuration   mConfiguration;
    private DispatcherImpl  mDispatcher;
    private Controller      mController;
    private Gamepad         mGamepad;
    private RobotTest       mRobot;

    @BeforeEach
    public void setUp() {
        try (MockedStatic<Environment> mockedEnvironment = Mockito.mockStatic(Environment.class)) {
            mockedEnvironment.when(Environment::getExternalStorageDirectory).thenReturn(new File(getClass().getClassLoader().getResource("results").getFile()));
            mLogger = new LogManager(null, null, "dispatcher-test",2);
            mLogger.level(LogManager.Severity.INFO);
            mLogger.info("Setting it up!");
        }
    }

    @AfterEach
    public void tearDown() {
        mLogger.stop();
    }

    @Test
    public void read() {

        mConfiguration  = new Configuration(mLogger);
        mGamepad        = new Gamepad();
        mController     = new Controller(mGamepad, mLogger);
        mRobot          = new RobotTest(mLogger);

        Map<String,Controller> controllers = new LinkedHashMap<>();
        controllers.put("1",mController);

        mDispatcher = new DispatcherImpl(controllers,mRobot, mLogger);
        mConfiguration.register("scheduler", mDispatcher);

        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/scheduling-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is invalid");

        mGamepad.left_stick_x   = -0.05f;
        mGamepad.left_stick_y   = -0.3f;
        mGamepad.right_stick_x  = -1.0f;
        mGamepad.left_trigger   = 0.4f;
        mGamepad.left_bumper    = true;
        mGamepad.right_bumper   = true;
        mGamepad.a              = true;
        mDispatcher.update();

        assertTrue(mRobot.test1Check(), "Scheduler should not have called test1");
        assertEquals(0.9,mRobot.test2Value1(), 0.0000001,"Scheduler should have called test2 with parameter 0.9");
        assertEquals(0.4,mRobot.test2Value2(), 0.0000001,"Scheduler should have called test2 with parameter 0.4");
        assertEquals(0.7,mRobot.commandValue3(), 0.0000001,"Scheduler should have called command with parameter 0.7");
        assertEquals(0.5,mRobot.commandValue4(), 0.0000001,"Scheduler should have called command with parameter 0.5");
        assertEquals(0.0,mRobot.x(), 0.0000001,"Scheduler should have called drive with parameter 0.0");
        assertEquals(-0.125,mRobot.y(), 0.0000001,"Scheduler should have called drive with parameter 0.125");
        assertEquals(-0.8,mRobot.heading(), 0.0000001,"Scheduler should have called drive with parameter -0.8");

        mRobot.reset();

    }

    @Test
    public void dynamic() {

        mConfiguration  = new Configuration(mLogger);
        mGamepad        = new Gamepad();
        mController     = new Controller(mGamepad, mLogger);
        mRobot          = new RobotTest(mLogger);

        Map<String,Controller> controllers = new LinkedHashMap<>();
        controllers.put("1",mController);

        mDispatcher = new DispatcherImpl(controllers,mRobot, mLogger);
        mConfiguration.register("scheduler", mDispatcher);

        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/scheduling-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is invalid");

        mGamepad.left_stick_x   = -0.05f;
        mGamepad.left_stick_y   = -0.3f;
        mGamepad.right_stick_x  = -1.0f;
        mGamepad.left_trigger   = 0.4f;
        mGamepad.left_bumper    = true;
        mGamepad.right_bumper   = true;
        mGamepad.a              = true;
        mDispatcher.update();

        assertTrue(mRobot.test1Check(), "Scheduler should not have called test1");
        assertEquals(0.9,mRobot.test2Value1(), 0.0000001,"Scheduler should have called test2 with parameter 0.9");
        assertEquals(0.4,mRobot.test2Value2(), 0.0000001,"Scheduler should have called test2 with parameter 0.4");
        assertEquals(0.7,mRobot.commandValue3(), 0.0000001,"Scheduler should have called command with parameter 0.7");
        assertEquals(0.5,mRobot.commandValue4(), 0.0000001,"Scheduler should have called command with parameter 0.5");
        assertEquals(0.0,mRobot.x(), 0.0000001,"Scheduler should have called drive with parameter 0.0");
        assertEquals(-0.125,mRobot.y(), 0.0000001,"Scheduler should have called drive with parameter 0.125");
        assertEquals(-0.8,mRobot.heading(), 0.0000001,"Scheduler should have called drive with parameter -0.8");

        mRobot.reset();

        mGamepad.left_stick_x   = -0.55f;
        mGamepad.left_stick_y   = 0.4f;
        mGamepad.right_stick_x  = 0.25f;
        mGamepad.left_trigger   = 0.4f;
        mGamepad.left_bumper    = true;
        mGamepad.right_bumper   = true;
        mGamepad.a              = true;
        mDispatcher.update();

        assertTrue(mRobot.test1Check(), "Scheduler should not have called test1");
        assertEquals(-10000,mRobot.test2Value1(), 0.0000001,"Scheduler should not have called test2");
        assertEquals(-10000,mRobot.test2Value2(), 0.0000001,"Scheduler should not have called test2");
        assertEquals(-10000,mRobot.commandValue3(), 0.0000001,"Scheduler should not have called command");
        assertEquals(-10000,mRobot.commandValue4(), 0.0000001,"Scheduler should not have called command");
        assertEquals(-0.55,mRobot.x(), 0.0000001,"Scheduler should have called test2 with parameter -0.55");
        assertEquals(0.25,mRobot.y(), 0.0000001,"Scheduler should have called test2 with parameter 0.25");
        assertEquals(0,mRobot.heading(), 0.0000001,"Scheduler should have called test2 with parameter 0");

        mRobot.reset();

        mGamepad.left_bumper    = false;
        mDispatcher.update();
        mRobot.reset();
        mGamepad.left_trigger   = 0.6f;
        mGamepad.left_bumper    = true;
        mDispatcher.update();
        assertEquals(0.6,mRobot.test2Value2(), 0.0000001,"Scheduler should have called test2 with parameter 0.4");
        mRobot.reset();



    }
}

class DispatcherImpl extends Dispatcher {

    public DispatcherImpl(Map<String,Controller> controllers, Robot robot, LogManager logger) {
        super(controllers, robot, logger);
    }

    @Override
    protected     void commands() {

        registerCommand(
                new Condition(() -> mControllers.get("1").buttons.left_bumper.pressedOnce()),
                () -> ((RobotTest)mRobot).test2(0.9,mControllers.get("1").axes.left_trigger.value()));



    }

}

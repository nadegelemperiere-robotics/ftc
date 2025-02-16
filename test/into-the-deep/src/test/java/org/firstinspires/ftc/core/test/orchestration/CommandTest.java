/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Configuration management test class
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.test.orchestration;

/* System includes */
import java.io.File;
import java.util.Map;
import java.util.LinkedHashMap;

/* Android includes */
import android.os.Environment;

/* Junit 5 includes */
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
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

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Component includes */
import org.firstinspires.ftc.core.components.controllers.Controller;

/* Controller includes */
import org.firstinspires.ftc.core.orchestration.scheduler.Command;

/* Tests includes */
import org.firstinspires.ftc.core.orchestration.test.RobotTest;

@ExtendWith(MockitoExtension.class)
public class CommandTest {

    private LogManager      mLogger;
    private Configuration   mConfiguration;
    private Command         mCommand;
    private Controller      mController;
    private Gamepad         mGamepad;
    private RobotTest       mRobot;

    @BeforeEach
    public void setUp() {
        try (MockedStatic<Environment> mockedEnvironment = Mockito.mockStatic(Environment.class)) {
            mockedEnvironment.when(Environment::getExternalStorageDirectory).thenReturn(new File(getClass().getClassLoader().getResource("results").getFile()));
            mLogger = new LogManager(null, null, "command-test");
            mLogger.level(LogManager.Severity.INFO);
            mLogger.info("Setting it up!");
        }
    }

    @Test
    public void read() {

        mConfiguration  = new Configuration(mLogger);
        mGamepad        = new Gamepad();
        mController     = new Controller(mGamepad, mLogger);
        mRobot          = new RobotTest(mLogger);

        Map<String,Controller> controllers = new LinkedHashMap<>();
        controllers.put("1",mController);

        mCommand = new Command(controllers,mRobot, mLogger);
        mConfiguration.register("command", mCommand);

        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/command-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is invalid");

        mGamepad.dpad_up = false;
        mGamepad.left_trigger = 0;
        mCommand.execute();
        assertFalse(mRobot.test1Check(), "Command should not have called test1");

    }

    @Test
    public void condition() {

        mConfiguration  = new Configuration(mLogger);
        mGamepad        = new Gamepad();
        mController     = new Controller(mGamepad, mLogger);
        mRobot          = new RobotTest(mLogger);

        Map<String,Controller> controllers = new LinkedHashMap<>();
        controllers.put("1",mController);

        mCommand = new Command(controllers,mRobot, mLogger);
        mConfiguration.register("command", mCommand);

        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/command-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is invalid");

        mGamepad.dpad_up = false;
        mGamepad.left_trigger = 0;
        mCommand.execute();
        assertFalse(mRobot.test1Check(), "Command should not have called test1");
        mRobot.reset();
        mGamepad.dpad_up = true;
        mGamepad.left_trigger = 0;
        mCommand.execute();
        assertFalse(mRobot.test1Check(), "Command should not have called test1");
        mRobot.reset();
        mGamepad.dpad_up = true;
        mGamepad.left_trigger = 1.0f;
        mCommand.execute();
        assertFalse(mRobot.test1Check(), "Command should not have called test1");
        mRobot.reset();
        mGamepad.dpad_up = false;
        mGamepad.left_trigger = 1.0f;
        mCommand.execute();
        assertFalse(mRobot.test1Check(), "Command should not have called test1");
        mRobot.reset();
        mGamepad.dpad_up = true;
        mGamepad.left_trigger = 1.0f;
        mCommand.execute();
        assertTrue(mRobot.test1Check(), "Command should have called test1");
        mRobot.reset();

    }

    @Test
    public void action() {

        mConfiguration  = new Configuration(mLogger);
        mGamepad        = new Gamepad();
        mController     = new Controller(mGamepad, mLogger);
        mRobot          = new RobotTest(mLogger);

        Map<String,Controller> controllers = new LinkedHashMap<>();
        controllers.put("1",mController);

        mCommand = new Command(controllers,mRobot, mLogger);
        mConfiguration.register("command", mCommand);

        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/command-2.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is invalid");

        mGamepad.dpad_up = true;
        mGamepad.left_trigger = 0.4f;
        mGamepad.right_trigger = 0.4f;
        mCommand.execute();
        assertEquals(0.9,mRobot.test2Value1(), 0.0000001,"Command should have called test2 with parameter 0.9");
        assertEquals(0.5*0.4,mRobot.test2Value2(), 0.0000001,"Command should have called test2 with parameter 0.5*0.4");
        mRobot.reset();
    }

}



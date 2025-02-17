/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Controller test class
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.test.robot;

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

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.Mock;

/* Component Under Test includes */
import org.firstinspires.ftc.intothedeep.v1.robot.Robot;

@ExtendWith(MockitoExtension.class)
public class RobotV1Test {

    private LogManager      mLogger;
    private Configuration   mConfiguration;
    private Robot           mRobot;

    @BeforeEach
    public void setUp() {
        try (MockedStatic<Environment> mockedEnvironment = Mockito.mockStatic(Environment.class)) {
            mockedEnvironment.when(Environment::getExternalStorageDirectory).thenReturn(new File(getClass().getClassLoader().getResource("results").getFile()));
            mLogger = new LogManager(null, null, "robot-v1-test",3);
            mLogger.level(LogManager.Severity.TRACE);
            mLogger.info("Setting it up!");
        }
    }

    @AfterEach
    public void tearDown() {
        mLogger.stop();
    }


    @Test
    public void read() {

        mConfiguration = new Configuration(mLogger);
        mRobot = new Robot(null, mLogger);
        mConfiguration.register("robot", mRobot);
        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/state-manager-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is valid");

    }

    @Test
    public void write() {

        mConfiguration = new Configuration(mLogger);
        mRobot = new Robot(null, mLogger);
        mConfiguration.register("robot", mRobot);
        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/state-manager-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is valid");

        mConfiguration.log();
        mLogger.update();

        mConfiguration.write(getClass().getClassLoader().getResource("results").getFile() + "/robot-v1-write.json");

    }

    @Test
    public void initState() {

        mConfiguration = new Configuration(mLogger);
        mRobot = new Robot(null, mLogger);
        mConfiguration.register("robot", mRobot);
        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/state-manager-1.json").getFile());
        Mock.instance().clear();

        assertTrue(mConfiguration.isValid(), "Configuration is valid");

        mRobot.start();
        mRobot.update();

        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-slides-power"),"Intake slides power shall have been mocked");
        assertEquals(0,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be 0");
        mRobot.powerIntakeSlides(1.0);
        mRobot.update();
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-slides-power"),"Intake slides power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall remain 0.0");

        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-slides-power"),"Outtake slides power shall have been mocked");
        assertEquals(0,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be 0");
        mRobot.powerOuttakeSlides(0.3);
        mRobot.update();
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-slides-power"),"Outtake slides power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall remain 0");

        assertTrue(Mock.instance().mMockedComponents.containsKey("front-left-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("back-left-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be 0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("front-right-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be 0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("back-right-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 0");
        mRobot.drive(1.0,1.0,0.0);
        mRobot.update();
        assertTrue(Mock.instance().mMockedComponents.containsKey("front-left-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power remain 0.0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("back-left-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power remain 0.0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("front-right-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power remain 0.0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("back-right-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power remain 0.0");
        mRobot.tuneDriveSpeed(0.6);
        mRobot.update();
        mRobot.drive(1.0,0.0,0.0);
        mRobot.update();
        assertTrue(Mock.instance().mMockedComponents.containsKey("front-left-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power remain 0.0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("back-left-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power remain 0.0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("front-right-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power remain 0.0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("back-right-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power remain 0.0");

        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-claw-position"),"Intake claw position shall have been mocked");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be open");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-elbow-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.66,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be positioned grab");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-arm-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake elbow shall be positioned transfer");
        mRobot.toggleIntakeClaw();
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-claw-position"),"Intake claw position shall have been mocked");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall remain open");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-elbow-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.66,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall remain in grab position");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-arm-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake elbow shall remain in transfer position");


        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-claw-position"),"Outtake claw position shall have been mocked");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be closed");
        mRobot.toggleOuttakeClaw();
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-claw-position"),"Outtake claw position shall have been mocked");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall remain closed");

        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-wrist-roll-position"),"Intake wrist position shall have been mocked");
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        mRobot.toggleIntakeWrist();
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-wrist-roll-position"),"Intake wrist position shall have been mocked");
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall remain oriented 0");

        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-elbow-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.66,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be positioned grab");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-arm-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake elbow shall be oriented transfer");
        mRobot.moveIntakeArm("down");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-wrist-roll-position"),"Intake wrist position shall have been mocked");
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be remain oriented 0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-elbow-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.66,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall remain in grab position");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-arm-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake elbow shall be remain in transfer position");
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-claw-position"),"Outtake claw position shall have been mocked");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall remain closed");


        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-elbow-pitch-position"),"Outtake elbow position shall have been mocked");
        assertEquals(0.08,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be positioned off");
        mRobot.moveOuttakeArm("up");
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-elbow-pitch-position"),"Outtake elbow position shall have been mocked");
        assertEquals(0.08,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall remain in off position");
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-wrist-roll-position"),"Outtake wrist position shall have been mocked");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall remain oriented 0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-claw-position"),"Outtake claw position shall have been mocked");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall remain closed");

        assertFalse(Mock.instance().mMockedComponents.containsKey("outtake-slides-position"),"Outtake slides position not shall have been mocked yet");
        mRobot.positionOuttakeSlides("autonomous-specimen-submersible-over");
        assertFalse(Mock.instance().mMockedComponents.containsKey("outtake-slides-position"),"Outtake slides position shall remain unmocked");

    }


    @Test
    public void defaultState() {

        mConfiguration = new Configuration(mLogger);
        mRobot = new Robot(null, mLogger);
        mConfiguration.register("robot", mRobot);
        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/state-manager-1.json").getFile());
        Mock.instance().clear();

        assertTrue(mConfiguration.isValid(), "Configuration is valid");

        mRobot.start();
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }

        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-slides-power"),"Intake slides power shall have been mocked");
        assertEquals(0,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be 0");
        mRobot.powerIntakeSlides(1.0);
        mRobot.update();
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-slides-power"),"Intake slides power shall have been mocked");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be 1.0");

        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-slides-power"),"Outtake slides power shall have been mocked");
        assertEquals(0,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be 0");
        mRobot.powerOuttakeSlides(0.3);
        mRobot.update();
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-slides-power"),"Outtake slides power shall have been mocked");
        assertEquals(0.3,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be 0.3");

        assertTrue(Mock.instance().mMockedComponents.containsKey("front-left-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("back-left-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be 0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("front-right-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be 0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("back-right-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 0");
        mRobot.drive(1.0,1.0,0.0);
        mRobot.update();
        assertTrue(Mock.instance().mMockedComponents.containsKey("front-left-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 1.0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("back-left-wheel-power"),"Wheel power shall have been mocked");
        assertTrue(Mock.instance().mMockedComponents.get("back-left-wheel-power") < 0,"Wheel power shall be negative");
        assertTrue(Math.abs(Mock.instance().mMockedComponents.get("back-left-wheel-power")) < 0.05,"Wheel power shall be low");
        assertTrue(Mock.instance().mMockedComponents.get("front-right-wheel-power") < 0,"Wheel power shall be negative");
        assertTrue(Math.abs(Mock.instance().mMockedComponents.get("front-right-wheel-power")) < 0.05,"Wheel power shall be low");
        assertTrue(Mock.instance().mMockedComponents.containsKey("back-right-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 1.0");
        mRobot.tuneDriveSpeed(0.6);
        mRobot.update();
        mRobot.drive(1.0,0.0,0.0);
        mRobot.update();
        assertTrue(Mock.instance().mMockedComponents.containsKey("front-left-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.6,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 0.6");
        assertTrue(Mock.instance().mMockedComponents.containsKey("back-left-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(-0.6,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be 0.6");
        assertTrue(Mock.instance().mMockedComponents.containsKey("front-right-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(-0.6,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be 0.6");
        assertTrue(Mock.instance().mMockedComponents.containsKey("back-right-wheel-power"),"Wheel power shall have been mocked");
        assertEquals(0.6,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 0.6");

        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-claw-position"),"Intake claw position shall have been mocked");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be open");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-elbow-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.66,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented grab");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-arm-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake elbow shall be oriented transfer");
        mRobot.toggleIntakeClaw();
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-wrist-roll-position"),"Intake wrist position shall have been mocked");
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-elbow-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.71,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented oversub");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-arm-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.6,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake elbow shall be oriented oversub");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-claw-position"),"Intake claw position shall have been mocked");
        assertEquals(0.62,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be closed");


        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-claw-position"),"Outtake claw position shall have been mocked");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be closed");
        mRobot.toggleOuttakeClaw();
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-claw-position"),"Outtake claw position shall have been mocked");
        assertEquals(0.36,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be open");

        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-wrist-roll-position"),"Intake wrist position shall have been mocked");
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        mRobot.toggleIntakeWrist();
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-wrist-roll-position"),"Intake wrist position shall have been mocked");
        assertEquals(0.675,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 180");

        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-elbow-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.71,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented oversub");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-arm-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.6,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake elbow shall be oriented oversub");
        mRobot.moveIntakeArm("down");
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-wrist-roll-position"),"Intake wrist position shall have been mocked");
        assertEquals(0.675,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 180");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-elbow-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.68,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented drone");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-arm-pitch-position"),"Intake elbow position shall have been mocked");
        assertEquals(0.44,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake elbow shall be oriented drone");
        assertTrue(Mock.instance().mMockedComponents.containsKey("intake-claw-position"),"Intake claw position shall have been mocked");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be open");


        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-elbow-pitch-position"),"Outtake elbow position shall have been mocked");
        assertEquals(0.08,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be positioned off");
        mRobot.moveOuttakeArm("up");
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-elbow-pitch-position"),"Outtake elbow position shall have been mocked");
        assertEquals(0.05,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be positioned drop sample");
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-wrist-roll-position"),"Outtake wrist position shall have been mocked");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall be oriented 0");
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-claw-position"),"Outtake claw position shall have been mocked");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be closed");

        mRobot.toggleOuttakeClaw();
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-claw-position"),"Outtake claw position shall have been mocked");
        assertEquals(0.36,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be open");
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-elbow-pitch-position"),"Outtake elbow position shall have been mocked");
        assertEquals(0.08,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be positioned off");
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-wrist-roll-position"),"Outtake wrist position shall have been mocked");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall be oriented 0");

        assertFalse(Mock.instance().mMockedComponents.containsKey("outtake-slides-position"),"Outtake slides position not shall have been mocked yet");
        mRobot.positionOuttakeSlides("autonomous-specimen-submersible-over");
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertTrue(Mock.instance().mMockedComponents.containsKey("outtake-slides-position"),"Outtake slides position shall have been mocked");
        assertEquals(870,Mock.instance().mMockedComponents.get("outtake-slides-position"),"Outtake slides position shall be 0");

    }


}

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

        Mock.instance().clear();

        mConfiguration = new Configuration(mLogger);
        mRobot = new Robot(null, mLogger);
        mConfiguration.register("robot", mRobot);
        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/state-manager-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is valid");

    }

    @Test
    public void write() {

        Mock.instance().clear();

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

        Mock.instance().clear();

        mConfiguration = new Configuration(mLogger);
        mRobot = new Robot(null, mLogger);
        mConfiguration.register("robot", mRobot);
        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/state-manager-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is valid");

        mRobot.start();
        mRobot.update();

        assertEquals(0.0,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be 0");
        mRobot.powerIntakeSlides(1.0);
        mRobot.update();
        assertEquals(0.0,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall remain 0.0");

        assertEquals(0,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be 0");
        mRobot.powerOuttakeSlides(0.3);
        mRobot.update();
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall remain 0");

        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 0");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be 0");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be 0");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 0");
        mRobot.drive(1.0,1.0,0.0);
        mRobot.update();
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power remain 0.0");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power remain 0.0");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power remain 0.0");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power remain 0.0");
        mRobot.tuneDriveSpeed(0.6);
        mRobot.update();
        mRobot.drive(1.0,0.0,0.0);
        mRobot.update();
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power remain 0.0");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power remain 0.0");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power remain 0.0");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power remain 0.0");

        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be open");
        assertEquals(0.66,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be positioned grab");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake elbow shall be positioned transfer");
        mRobot.toggleIntakeClaw();
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall remain open");
        assertEquals(0.66,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall remain in grab position");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall remain in transfer position");

        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be closed");
        mRobot.toggleOuttakeClaw();
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall remain closed");

        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        mRobot.toggleIntakeWrist();
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall remain oriented 0");

        assertEquals(0.66,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be positioned grab");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall be oriented transfer");
        mRobot.moveIntakeArm("down");
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be remain oriented 0");
        assertEquals(0.66,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall remain in grab position");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall be remain in transfer position");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall remain closed");

        assertEquals(0.08,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be positioned off");
        mRobot.moveOuttakeArm("up");
        assertEquals(0.08,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall remain in off position");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall remain oriented 0");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall remain closed");

        assertFalse(Mock.instance().mMockedComponents.containsKey("outtake-slides-position"),"Outtake slides position not shall have been mocked yet");
        mRobot.positionOuttakeSlides("autonomous-specimen-submersible-over");
        assertFalse(Mock.instance().mMockedComponents.containsKey("outtake-slides-position"),"Outtake slides position shall remain unmocked");

    }

    @Test
    public void defaultState() {

        Mock.instance().clear();

        mConfiguration = new Configuration(mLogger);
        mRobot = new Robot(null, mLogger);
        mConfiguration.register("robot", mRobot);
        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/state-manager-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is valid");

        // Start and give time to reach default mode
        mRobot.start();
        while(!mRobot.state().equals("DefaultState")) {
            mRobot.update();
            try { Thread.sleep(50); } catch (InterruptedException ignored) { }
        }

        assertEquals(0,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be 0");
        mRobot.powerIntakeSlides(1.0);
        mRobot.update();
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be 1.0");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-slides-mode"),"Intake slides shall be in RUN_USING_ENCODER mode");

        assertEquals(0,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be 0");
        mRobot.powerOuttakeSlides(0.3);
        mRobot.update();
        assertEquals(0.3,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be 0.3");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("outtake-slides-mode"),"Intake slides shall be in RUN_USING_ENCODER mode");

        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 0");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be 0");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be 0");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 0");
        mRobot.drive(1.0,1.0,0.0);
        mRobot.update();
        assertEquals(1.0,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 1.0");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("front-left-wheel-mode"),"Wheel shall be in RUN_USING_ENCODER mode");
        assertTrue(Mock.instance().mMockedComponents.get("back-left-wheel-power") < 0,"Wheel power shall be negative");
        assertTrue(Math.abs(Mock.instance().mMockedComponents.get("back-left-wheel-power")) < 0.05,"Wheel power shall be low");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("back-left-wheel-mode"),"Wheel shall be in RUN_USING_ENCODER mode");
        assertTrue(Mock.instance().mMockedComponents.get("front-right-wheel-power") < 0,"Wheel power shall be negative");
        assertTrue(Math.abs(Mock.instance().mMockedComponents.get("front-right-wheel-power")) < 0.05,"Wheel power shall be low");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("front-right-wheel-mode"),"Wheel shall be in RUN_USING_ENCODER mode");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 1.0");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("back-right-wheel-mode"),"Wheel shall be in RUN_USING_ENCODER mode");
        mRobot.tuneDriveSpeed(0.6);
        mRobot.update();
        mRobot.drive(1.0,0.0,0.0);
        mRobot.update();
        assertEquals(0.6,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 0.6");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("front-left-wheel-mode"),"Wheel shall be in RUN_USING_ENCODER mode");
        assertEquals(-0.6,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be 0.6");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("back-left-wheel-mode"),"Wheel shall be in RUN_USING_ENCODER mode");
        assertEquals(-0.6,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be 0.6");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("front-right-wheel-mode"),"Wheel shall be in RUN_USING_ENCODER mode");
        assertEquals(0.6,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 0.6");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("back-right-wheel-mode"),"Wheel shall be in RUN_USING_ENCODER mode");

        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be open");
        assertEquals(0.66,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented grab");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall be oriented transfer");
        mRobot.toggleIntakeClaw();
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        assertEquals(0.71,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented oversub");
        assertEquals(0.6,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall be oriented oversub");
        assertEquals(0.62,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be closed");

        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be closed");
        mRobot.toggleOuttakeClaw();
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertEquals(0.36,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be open");

        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        mRobot.toggleIntakeWrist();
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertEquals(0.675,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 180");

        assertEquals(0.71,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented oversub");
        assertEquals(0.6,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake elbow shall be oriented oversub");
        mRobot.moveIntakeArm("down");
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertEquals(0.675,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 180");
        assertEquals(0.68,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented drone");
        assertEquals(0.44,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake elbow shall be oriented drone");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be open");

        assertEquals(0.08,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be positioned off");
        mRobot.moveOuttakeArm("up");
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertEquals(0.05,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be positioned drop sample");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall be oriented 0");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be closed");

        mRobot.toggleOuttakeClaw();
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertEquals(0.36,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be open");
        assertEquals(0.08,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be positioned off");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall be oriented 0");

        assertFalse(Mock.instance().mMockedComponents.containsKey("outtake-slides-position"),"Outtake slides position not shall have been mocked yet");
        mRobot.positionOuttakeSlides("autonomous-specimen-submersible-over");
        for(int i_time = 0; i_time < 50; i_time ++) {
            mRobot.update();
            try { Thread.sleep(20); } catch (InterruptedException ignored) { }
        }
        assertEquals(870,Mock.instance().mMockedComponents.get("outtake-slides-position"),"Outtake slides position shall be 0");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-mode"),"Intake slides shall be in RUN_TO_POSITION");

    }

    @Test
    public void transferState() {

        Mock.instance().clear();

        mConfiguration = new Configuration(mLogger);
        mRobot = new Robot(null, mLogger);
        mConfiguration.register("robot", mRobot);
        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/state-manager-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is valid");

        // Start and wait to reach default state
        mRobot.start();
        while(!mRobot.state().equals("DefaultState")) {
            mRobot.update();
            try { Thread.sleep(50); } catch (InterruptedException ignored) { }
        }

        // Launch all motors
        mRobot.powerIntakeSlides(1.0);
        mRobot.powerOuttakeSlides(0.3);
        mRobot.drive(1.0,0.0,0.0);
        mRobot.update();

        // Check motors all have power
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be 1.0");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-slides-mode"),"Intake slides shall be in RUN_USING_ENCODER");
        assertEquals(0.3,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be 0.3");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("outtake-slides-mode"),"Outtake slides shall be in RUN_USING_ENCODER");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 1.0");
        assertEquals(-1.0,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be 1.0");
        assertEquals(-1.0,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be 1.0");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 1.0");

        // Launch transfer
        mRobot.transfer();
        assertEquals("TransferState",mRobot.state(),"Robot should be in transfer state");
        // Execute all possible commands - only drive related commands should have impact
        mRobot.powerIntakeSlides(0.6);
        mRobot.powerOuttakeSlides(0.6);
        mRobot.tuneDriveSpeed(0.6);
        mRobot.drive(-1.0,0.0,0.0);
        mRobot.toggleIntakeClaw();
        mRobot.toggleOuttakeClaw();
        mRobot.toggleIntakeWrist();
        mRobot.moveIntakeArm("down");
        mRobot.moveOuttakeArm("up");
        mRobot.positionOuttakeSlides("autonomous-specimen-submersible-over");

        // Mechanisms motors should have been stopped
        assertEquals(0.0,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be 0.0");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("outtake-slides-mode"),"Intake slides shall be in RUN_USING_ENCODER");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be 0.0");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("outtake-slides-mode"),"Intake slides shall be in RUN_USING_ENCODER");

        // Drive motors should be on, and the commands should have succeeded
        assertEquals(-0.6,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 0.0");
        assertEquals(0.6,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be 0.0");
        assertEquals(0.6,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be 0.0");
        assertEquals(-0.6,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 0.0");

        // Mechanisms should not have been impacted by commands
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        assertEquals(0.66,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented grab");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall be oriented transfer");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be open");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall be oriented 0");
        assertEquals(0.08,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be oriented off");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be closed");

        // Step to intake slides and outtake slides moving
        mRobot.update();
        assertEquals("TransferState",mRobot.state(),"Robot should be in transfer state");

        // Execute all possible commands - only drive related commands should have impact
        mRobot.powerIntakeSlides(0.6);
        mRobot.powerOuttakeSlides(0.6);
        mRobot.tuneDriveSpeed(0.5);
        mRobot.drive(1.0,0.0,0.0);
        mRobot.toggleIntakeClaw();
        mRobot.toggleOuttakeClaw();
        mRobot.toggleIntakeWrist();
        mRobot.moveIntakeArm("down");
        mRobot.moveOuttakeArm("up");
        mRobot.positionOuttakeSlides("autonomous-specimen-submersible-over");

        // Intake slides and outtake slides shall have powers and position to hold
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be the set-position one");
        assertEquals(300,Mock.instance().mMockedComponents.get("intake-slides-position"),"Intake slides position should be transfer-away");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("intake-slides-mode"),"Intake slides shall be in RUN_TO_POSITION");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be the set-position one");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-position"),"Intake slides position should be transfer");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-mode"),"Outtake slides shall be in RUN_TO_POSITION");

        // Drive motors should be on, and the commands should have succeeded
        assertEquals(0.5,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 0.0");
        assertEquals(-0.5,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be 0.0");
        assertEquals(-0.5,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be 0.0");
        assertEquals(0.5,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 0.0");

        // Mechanisms should not have been impacted by commands
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        assertEquals(0.66,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented grab");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall be oriented transfer");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be open");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall be oriented 0");
        assertEquals(0.08,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be oriented off");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be closed");

        // Step to arms moving - should be one update step since the mocked motors reach position instantaneously
        mRobot.update();
        assertEquals("TransferState",mRobot.state(),"Robot should be in transfer state");

        // Execute all possible commands - only drive related commands should have impact
        mRobot.powerIntakeSlides(0.6);
        mRobot.powerOuttakeSlides(0.6);
        mRobot.tuneDriveSpeed(0.4);
        mRobot.drive(-1.0,0.0,0.0);
        mRobot.toggleIntakeClaw();
        mRobot.toggleOuttakeClaw();
        mRobot.toggleIntakeWrist();
        mRobot.moveIntakeArm("down");
        mRobot.moveOuttakeArm("up");
        mRobot.positionOuttakeSlides("autonomous-specimen-submersible-over");

        // Slides shall hold their previous position with decreased power
        assertEquals(0.3,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be the hold-position one");
        assertEquals(300,Mock.instance().mMockedComponents.get("intake-slides-position"),"Intake slides position should be transfer-away");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("intake-slides-mode"),"Intake slides shall be in RUN_TO_POSITION");
        assertEquals(0.3,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be the hold-position one");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-position"),"Outtake slides position should be transfer");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-mode"),"Outtake slides shall be in RUN_TO_POSITION");

        // Drive motors should be on, and the commands should have succeeded
        assertEquals(-0.4,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be -0.4");
        assertEquals(0.4,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be 0.4");
        assertEquals(0.4,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be 0.4");
        assertEquals(-0.4,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be -0.4");

        // Mechanisms should be in transfer position
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        assertEquals(0.15,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented transfer");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall be oriented transfer");
        assertEquals(0.62,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be closed");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall be oriented 0");
        assertEquals(0.11,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be oriented transfer");
        assertEquals(0.36,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be open");

        // Step to microrelease - should be roughly 500 ms
        try { Thread.sleep(500); } catch (InterruptedException ignored) { }
        mRobot.update();
        assertEquals("TransferState",mRobot.state(),"Robot should be in transfer state");

        // Execute all possible commands - only drive related commands should have impact
        mRobot.powerIntakeSlides(0.6);
        mRobot.powerOuttakeSlides(0.7);
        mRobot.tuneDriveSpeed(0.7);
        mRobot.drive(1.0,0.0,0.0);
        mRobot.toggleIntakeClaw();
        mRobot.toggleOuttakeClaw();
        mRobot.toggleIntakeWrist();
        mRobot.moveIntakeArm("down");
        mRobot.moveOuttakeArm("up");
        mRobot.positionOuttakeSlides("autonomous-specimen-submersible-over");

        // Slides shall hold their previous position with decreased power
        assertEquals(0.3,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be the hold-position one");
        assertEquals(300,Mock.instance().mMockedComponents.get("intake-slides-position"),"Intake slides position should be transfer-away");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("intake-slides-mode"),"Intake slides shall be in RUN_TO_POSITION");
        assertEquals(0.3,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be the hold-position one");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-position"),"Outtake slides position should be transfer");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-mode"),"Outtake slides shall be in RUN_TO_POSITION");

        // Drive motors should be on, and the commands should have succeeded
        assertEquals(0.7,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 0.7");
        assertEquals(-0.7,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be -0.7");
        assertEquals(-0.7,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be -0.7");
        assertEquals(0.7,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 0.7");

        // Mechanisms should be in transfer position and inttake claw microreleased
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        assertEquals(0.15,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented transfer");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall be oriented transfer");
        assertEquals(0.9,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be microreleased");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall be oriented 0");
        assertEquals(0.11,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be oriented transfer");
        assertEquals(0.36,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be open");

        // Step to inttake claw close - should be roughly 500 ms
        try { Thread.sleep(500); } catch (InterruptedException ignored) { }
        mRobot.update();
        assertEquals("TransferState",mRobot.state(),"Robot should be in transfer state");

        // Execute all possible commands - only drive related commands should have impact
        mRobot.powerIntakeSlides(0.6);
        mRobot.powerOuttakeSlides(0.8);
        mRobot.tuneDriveSpeed(0.8);
        mRobot.drive(-1.0,0.0,0.0);
        mRobot.toggleIntakeClaw();
        mRobot.toggleOuttakeClaw();
        mRobot.toggleIntakeWrist();
        mRobot.moveIntakeArm("down");
        mRobot.moveOuttakeArm("up");
        mRobot.positionOuttakeSlides("autonomous-specimen-submersible-over");

        // Slides shall hold their previous position with decreased power
        assertEquals(0.3,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be the hold-position one");
        assertEquals(300,Mock.instance().mMockedComponents.get("intake-slides-position"),"Intake slides position should be transfer-away");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("intake-slides-mode"),"Intake slides shall be in RUN_TO_POSITION");
        assertEquals(0.3,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be the hold-position one");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-position"),"Outtake slides position should be transfer");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-mode"),"Outtake slides shall be in RUN_TO_POSITION");

        // Drive motors should be on, and the commands should have succeeded
        assertEquals(-0.8,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 0.7");
        assertEquals(0.8,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be -0.7");
        assertEquals(0.8,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be -0.7");
        assertEquals(-0.8,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 0.7");

        // Mechanisms should be in transfer position and inttake claw closed
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        assertEquals(0.15,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented transfer");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall be oriented transfer");
        assertEquals(0.62,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be closed");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall be oriented 0");
        assertEquals(0.11,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be oriented transfer");
        assertEquals(0.36,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be open");

        // Step to intake slides moving - should be roughly 500 ms
        try { Thread.sleep(500); } catch (InterruptedException ignored) { }
        mRobot.update();
        assertEquals("TransferState",mRobot.state(),"Robot should be in transfer state");

        // Execute all possible commands - only drive related commands should have impact
        mRobot.powerIntakeSlides(0.6);
        mRobot.powerOuttakeSlides(0.8);
        mRobot.tuneDriveSpeed(0.8);
        mRobot.drive(-1.0,0.0,0.0);
        mRobot.toggleIntakeClaw();
        mRobot.toggleOuttakeClaw();
        mRobot.toggleIntakeWrist();
        mRobot.moveIntakeArm("down");
        mRobot.moveOuttakeArm("up");
        mRobot.positionOuttakeSlides("autonomous-specimen-submersible-over");

        // Intake slides should be moving full speed and outtakelides shall hold their previous position with decreased power
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be the set-position one");
        assertEquals(167,Mock.instance().mMockedComponents.get("intake-slides-position"),"Intake slides position should be transfer-exchange");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("intake-slides-mode"),"Intake slides shall be in RUN_TO_POSITION");
        assertEquals(0.3,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be the hold-position one");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-position"),"Outtake slides position should be transfer");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-mode"),"Outtake slides shall be in RUN_TO_POSITION");

        // Drive motors should be on, and the commands should have succeeded
        assertEquals(-0.8,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 0.7");
        assertEquals(0.8,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be -0.7");
        assertEquals(0.8,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be -0.7");
        assertEquals(-0.8,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 0.7");

        // Mechanisms should be in transfer position
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        assertEquals(0.15,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented transfer");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall be oriented transfer");
        assertEquals(0.62,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be closed");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall be oriented 0");
        assertEquals(0.11,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be oriented transfer");
        assertEquals(0.36,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be open");

        // Step to outtake claw closing - should be instantaneous, because mock motor change position right away
        mRobot.update();
        assertEquals("TransferState",mRobot.state(),"Robot should be in transfer state");

        // Execute all possible commands - only drive related commands should have impact
        mRobot.powerIntakeSlides(0.6);
        mRobot.powerOuttakeSlides(0.8);
        mRobot.tuneDriveSpeed(0.7);
        mRobot.drive(1.0,0.0,0.0);
        mRobot.toggleIntakeClaw();
        mRobot.toggleOuttakeClaw();
        mRobot.toggleIntakeWrist();
        mRobot.moveIntakeArm("down");
        mRobot.moveOuttakeArm("up");
        mRobot.positionOuttakeSlides("autonomous-specimen-submersible-over");

        // Slides shall hold their previous position with decreased power
        assertEquals(0.3,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be the hold-position one");
        assertEquals(167,Mock.instance().mMockedComponents.get("intake-slides-position"),"Intake slides position should be transfer-exchange");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("intake-slides-mode"),"Intake slides shall be in RUN_TO_POSITION");
        assertEquals(0.3,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be the hold-position one");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-position"),"Outtake slides position should be transfer");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-mode"),"Outtake slides shall be in RUN_TO_POSITION");

        // Drive motors should be on, and the commands should have succeeded
        assertEquals(0.7,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be 0.7");
        assertEquals(-0.7,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be -0.7");
        assertEquals(-0.7,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be -0.7");
        assertEquals(0.7,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be 0.7");

        // Mechanisms should be in transfer position and outtake claw closed
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        assertEquals(0.15,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented transfer");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall be oriented transfer");
        assertEquals(0.62,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be closed");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall be oriented 0");
        assertEquals(0.11,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be oriented transfer");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be closed");

        // Step to intake claw opening - should be roughly 500 ms
        try { Thread.sleep(500); } catch (InterruptedException ignored) { }
        mRobot.update();
        assertEquals("TransferState",mRobot.state(),"Robot should be in transfer state");

        // Execute all possible commands - only drive related commands should have impact
        mRobot.powerIntakeSlides(0.6);
        mRobot.powerOuttakeSlides(0.8);
        mRobot.tuneDriveSpeed(0.3);
        mRobot.drive(-1.0,0.0,0.0);
        mRobot.toggleIntakeClaw();
        mRobot.toggleOuttakeClaw();
        mRobot.toggleIntakeWrist();
        mRobot.moveIntakeArm("down");
        mRobot.moveOuttakeArm("up");
        mRobot.positionOuttakeSlides("autonomous-specimen-submersible-over");

        // Slides shall hold their previous position with decreased power
        assertEquals(0.3,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides power shall be the hold-position one");
        assertEquals(167,Mock.instance().mMockedComponents.get("intake-slides-position"),"Intake slides position should be transfer-exchange");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("intake-slides-mode"),"Intake slides shall be in RUN_TO_POSITION");
        assertEquals(0.3,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides power shall be the hold-position one");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-position"),"Outtake slides position should be transfer");
        assertEquals(0.0,Mock.instance().mMockedComponents.get("outtake-slides-mode"),"Outtake slides shall be in RUN_TO_POSITION");

        // Drive motors should be on, and the commands should have succeeded
        assertEquals(-0.3,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be -0.3");
        assertEquals(0.3,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be 0.3");
        assertEquals(0.3,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be 0.3");
        assertEquals(-0.3,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be -0.3");

        // Mechanisms should be in transfer position and intake claw open
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        assertEquals(0.15,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented transfer");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall be oriented transfer");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be open");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall be oriented 0");
        assertEquals(0.11,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be oriented transfer");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be closed");

        // Step to release slides - should be roughly 500 ms
        try { Thread.sleep(500); } catch (InterruptedException ignored) { }
        mRobot.update();
        assertEquals("TransferState",mRobot.state(),"Robot should be in transfer state");

        // Execute all possible commands - only drive related commands should have impact
        mRobot.powerIntakeSlides(0.6);
        mRobot.powerOuttakeSlides(0.8);
        mRobot.tuneDriveSpeed(0.3);
        mRobot.drive(-1.0,0.0,0.0);
        mRobot.toggleIntakeClaw();
        mRobot.toggleOuttakeClaw();
        mRobot.toggleIntakeWrist();
        mRobot.moveIntakeArm("down");
        mRobot.moveOuttakeArm("up");
        mRobot.positionOuttakeSlides("autonomous-specimen-submersible-over");

        // Slides shall hold their previous position with decreased power
        assertEquals(0.05,Mock.instance().mMockedComponents.get("intake-slides-power"),"Intake slides shall have been freed");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-slides-mode"),"Intake slides shall be in RUN_USING_ENCODER");
        assertEquals(0.05,Mock.instance().mMockedComponents.get("outtake-slides-power"),"Outtake slides shall have been freed");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("outtake-slides-mode"),"Outtake slides shall be in RUN_USING_ENCODER");

        // Drive motors should be on, and the commands should have succeeded
        assertEquals(-0.3,Mock.instance().mMockedComponents.get("front-left-wheel-power"),"Wheel power shall be -0.3");
        assertEquals(0.3,Mock.instance().mMockedComponents.get("back-left-wheel-power"),"Wheel power shall be 0.3");
        assertEquals(0.3,Mock.instance().mMockedComponents.get("front-right-wheel-power"),"Wheel power shall be 0.3");
        assertEquals(-0.3,Mock.instance().mMockedComponents.get("back-right-wheel-power"),"Wheel power shall be -0.3");

        // Mechanisms should be in transfer position and intake claw open
        assertEquals(0.405,Mock.instance().mMockedComponents.get("intake-wrist-roll-position"),"Intake wrist shall be oriented 0");
        assertEquals(0.15,Mock.instance().mMockedComponents.get("intake-elbow-pitch-position"),"Intake elbow shall be oriented transfer");
        assertEquals(0.97,Mock.instance().mMockedComponents.get("intake-arm-pitch-position"),"Intake arm shall be oriented transfer");
        assertEquals(1.0,Mock.instance().mMockedComponents.get("intake-claw-position"),"Intake claw shall be open");
        assertEquals(0.135,Mock.instance().mMockedComponents.get("outtake-wrist-roll-position"),"Outtake wrist shall be oriented 0");
        assertEquals(0.11,Mock.instance().mMockedComponents.get("outtake-elbow-pitch-position"),"Outtake elbow shall be oriented transfer");
        assertEquals(0.73,Mock.instance().mMockedComponents.get("outtake-claw-position"),"Outtake claw shall be closed");

        // End of the transfer - Switching back to default state
        mRobot.update();
        assertEquals("DefaultState",mRobot.state(),"Robot should be back in default state");



    }


}

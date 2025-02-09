/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Logging manager tests
   ------------------------------------------------------- */

package org.firstinspires.ftc.core;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Acme robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;

/* Component Under Test includes */
import org.firstinspires.ftc.core.tools.LogManager;

@TeleOp(name = "LogManagerTest", group = "Test")
public class LogManagerTest extends LinearOpMode {

    private enum Suite {
        NONE,
        CONSTRUCTOR
    }

    Suite   mCurrentSuite;
    Suite   mNextSuite;

    LogManager mLogger;

    public void runOpMode() {

        telemetry.clear();
        telemetry.addLine("----- LOGGER TESTS -----");
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().clear();
        FtcDashboard.getInstance().getTelemetry().addLine("----- LOGGER TESTS -----");
        FtcDashboard.getInstance().getTelemetry().update();
        mCurrentSuite = Suite.NONE;
        mNextSuite = Suite.CONSTRUCTOR;

        waitForStart();

        while (opModeIsActive()) {

            if(mNextSuite != mCurrentSuite) {
                this.launch(mNextSuite);
                mNextSuite = mCurrentSuite;
            }

            sleep(200);
        }

    }

    private void launch(Suite suite) {
        if(suite == Suite.CONSTRUCTOR) { this.constructorTest(); }
        if(suite == Suite.CONSTRUCTOR) { this.constructorTest(); }
        else {
            telemetry.addLine("Unknown suite " + suite);
            FtcDashboard.getInstance().getTelemetry().addLine("Unknown suite " + suite);
        }
    }

    private void constructorTest() {

        telemetry.addLine("--> Constructor test\n");
        FtcDashboard.getInstance().getTelemetry().addLine("--> Constructor test");

        telemetry.addLine("----> Driver station only");
        FtcDashboard.getInstance().getTelemetry().addLine("----> Driver station only");

        mLogger = new LogManager(telemetry, null,"");
        for (LogManager.Target target : LogManager.Target.values()) {
            mLogger.error(target, "Error shall only appear on driver station");
            mLogger.warning(target, "Warning shall only appear on driver station");
            mLogger.metric("Target","DriverStation");
        }
        mLogger.stop();

        telemetry.addLine("----> Dashboard only");
        FtcDashboard.getInstance().getTelemetry().addLine("----> Dashboard only");

        mLogger = new LogManager(null, FtcDashboard.getInstance(),"");
        for (LogManager.Target target : LogManager.Target.values()) {
            mLogger.error(target, "Error line shall only appear on dashboard");
            mLogger.warning(target, "Warning shall only appear on dashboard");
            mLogger.metric("Target","Dashboard");
        }
        mLogger.stop();

        telemetry.addLine("----> File only");
        FtcDashboard.getInstance().getTelemetry().addLine("----> File only");

        mLogger = new LogManager(null, null,"log-manager");
        for (LogManager.Target target : LogManager.Target.values()) {
            mLogger.error(target, "Error line shall only appear in file");
            mLogger.warning(target, "Warning line shall only appear in file");
            mLogger.metric("Target","File");
        }
        mLogger.stop();

        telemetry.addLine("----> Nowhere");
        FtcDashboard.getInstance().getTelemetry().addLine("----> Nowhere");

        mLogger = new LogManager(null, null, "");
        for (LogManager.Target target : LogManager.Target.values()) {
            mLogger.error(target, "Error line shall not appear");
            mLogger.warning(target, "Warning line shall not appear");
            mLogger.metric("Target","None");
        }
        mLogger.stop();

        telemetry.addLine("----> Driver station, file and dashboard");
        FtcDashboard.getInstance().getTelemetry().addLine("----> Driver station and dashboard");

        mLogger = new LogManager(telemetry, FtcDashboard.getInstance(),"log-manager");
        for (LogManager.Target target : LogManager.Target.values()) {
            mLogger.error(target, "Error line shall appear on driver station, file and dashboard");
            mLogger.warning(target, "Warning line shall appear on driver station, file and dashboard");
            mLogger.metric("Target","All");
        }

        for (LogManager.Target target : LogManager.Target.values()) {
            mLogger.update(target);
        }


    }



}

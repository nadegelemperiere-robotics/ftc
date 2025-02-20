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
            mLogger.metric(target, "Target","DriverStation");
            mLogger.info(target, "Info shall only appear on driver station");
            mLogger.debug(target, "Debug shall only appear on driver station");
            mLogger.trace(target, "Trace shall only appear on driver station");
        }
        mLogger.stop();

        telemetry.addLine("----> Dashboard only");
        FtcDashboard.getInstance().getTelemetry().addLine("----> Dashboard only");

        mLogger = new LogManager(null, FtcDashboard.getInstance(),"");
        for (LogManager.Target target : LogManager.Target.values()) {
            mLogger.error(target, "Error line shall only appear on dashboard");
            mLogger.warning(target, "Warning shall only appear on dashboard");
            mLogger.metric(target, "Target","Dashboard");
            mLogger.info(target, "Info shall only appear on dashboard");
            mLogger.debug(target, "Debug shall only appear on dashboard");
            mLogger.trace(target, "Trace shall only appear on dashboard");
        }
        mLogger.stop();

        telemetry.addLine("----> File only");
        FtcDashboard.getInstance().getTelemetry().addLine("----> File only");

        mLogger = new LogManager(null, null,"log-manager-only");
        for (LogManager.Target target : LogManager.Target.values()) {
            mLogger.error(target, "Error line shall only appear in file");
            mLogger.warning(target, "Warning line shall only appear in file");
            mLogger.metric(target, "Target","File");
            mLogger.info(target, "Info shall only appear in file");
            mLogger.debug(target, "Debug shall only appear in file");
            mLogger.trace(target, "Trace shall only appear in file");
        }
        mLogger.stop();

        telemetry.addLine("----> Nowhere");
        FtcDashboard.getInstance().getTelemetry().addLine("----> Nowhere");

        mLogger = new LogManager(null, null, "");
        for (LogManager.Target target : LogManager.Target.values()) {
            mLogger.error(target, "Error line shall not appear");
            mLogger.warning(target, "Warning line shall not appear");
            mLogger.metric(target, "Target","None");
            mLogger.info(target, "Info line shall not appear");
            mLogger.debug(target, "Debug line shall not appear");
            mLogger.trace(target, "Trace line shall not appear");
        }
        mLogger.stop();

        telemetry.addLine("----> Driver station, file and dashboard");
        FtcDashboard.getInstance().getTelemetry().addLine("----> Driver station and dashboard");

        mLogger = new LogManager(telemetry, FtcDashboard.getInstance(),"log-manager-all");
        for (LogManager.Target target : LogManager.Target.values()) {
            mLogger.error(target, "Error line shall appear on driver station, file and dashboard");
            mLogger.warning(target, "Warning line shall appear on driver station, file and dashboard");
            mLogger.metric(target, "Target","All");
            mLogger.info(target, "Info line shall appear on driver station, file and dashboard");
            mLogger.debug(target, "Debug line shall appear on driver station, file and dashboard");
            mLogger.trace(target, "Trace line shall appear on driver station, file and dashboard");
        }
        mLogger.stop();

        for (LogManager.Target target : LogManager.Target.values()) {
            mLogger.update(target);
        }


    }



}

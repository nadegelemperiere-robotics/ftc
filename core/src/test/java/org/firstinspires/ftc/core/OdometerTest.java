/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Odometers localization test mode
   ------------------------------------------------------- */

package org.firstinspires.ftc.core;

/* System includes */
import android.os.Environment;

import java.util.LinkedHashMap;
import java.util.Map;

/* Qualcomm includes */
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* JSON includes */
import org.firstinspires.ftc.core.subsystems.Subsystem;
import org.json.JSONObject;

/* ACME robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Tools includes */
import org.firstinspires.ftc.core.subsystems.MecanumDrive;
import org.firstinspires.ftc.core.tools.LogManager;
import org.firstinspires.ftc.core.tools.Condition;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Components includes */
import org.firstinspires.ftc.core.components.controllers.Controller;

/* Subsystem includes */
import org.firstinspires.ftc.core.subsystems.DriveTrain;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Robot;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.dispatcher.Dispatcher;

@Config
@TeleOp(name = "OdometerTest", group = "Test")
public class OdometerTest extends OpMode {

    public static String    CONFIGURATION   = "test";

    LogManager              mLogger;

    Configuration           mConfiguration;
    String                  mConfigurationName;

    Map<String, Controller> mControllers;
    OdometerTestRobot       mRobot;
    OdometerTestDispatcher  mDispatcher;

    @Override
    public void init(){

        try {

            // Log initialization
            mLogger = new LogManager(telemetry, FtcDashboard.getInstance(),"odometer");
            mLogger.level(LogManager.Severity.INFO);
            mLogger.clear();

            // Configuration initialization
            mConfigurationName  = CONFIGURATION;
            mConfiguration      = Configuration.getInstance();
            mConfiguration.logger(mLogger);

            // Robot initialization
            mRobot = new OdometerTestRobot(hardwareMap, mLogger);

            // Control initialization
            mControllers = new LinkedHashMap<>();
            mControllers.put("drive", new Controller(gamepad1, mLogger));
            mControllers.put("mechanisms", new Controller(gamepad2, mLogger));
            mDispatcher = new OdometerTestDispatcher(mControllers, mRobot, mLogger);

            // Register configurables
            mConfiguration.register("robot", mRobot);
            mConfiguration.read(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/" + mConfigurationName + ".json");
            mConfiguration.log();

            mLogger.update();

        }
        catch(Exception e){
            mLogger.error(e.getMessage());
            mConfiguration.log();
            mLogger.update();
        }

    }

    @Override
    public void start() {
        mLogger.reset();
        mRobot.start();
    }

    @Override
    public void loop (){

        try {

            mLogger.info(LogManager.Target.FILE, "1- Update robot");
            mRobot.update();
            mLogger.info(LogManager.Target.FILE, "2- Log robot");
            mRobot.log();
            mLogger.info(LogManager.Target.FILE, "3- Update dispatcher");
            mDispatcher.update();
            mLogger.info(LogManager.Target.FILE, "4- Update logs");
            mLogger.update();
            mLogger.info(LogManager.Target.FILE, "5- End");

        }
        catch(Exception e){
            mLogger.error(e.getMessage());
            mLogger.update();
        }

    }

    @Override
    public void stop() {
        mRobot.persist();
        mLogger.update();
        mLogger.stop();
    }

}


class OdometerTestDispatcher extends Dispatcher {

    public OdometerTestDispatcher(Map<String,Controller> controllers, OdometerTestRobot robot, LogManager logger) {
        super(controllers, robot, logger);
    }

    protected void commands() {
        this.registerCommand(
                new Condition(() -> true),
                () -> {
                    ((OdometerTestRobot) mRobot).drive(
                            mControllers.get("drive").axes.left_stick_x.value(),
                            mControllers.get("drive").axes.left_stick_y.value(),
                            mControllers.get("drive").axes.right_stick_x.value());
                }
        );
    }

}

class OdometerTestRobot extends Robot {

    static final String sChassisKey = "drive-train";

    DriveTrain          mChassis;

    public  OdometerTestRobot(HardwareMap map, LogManager logger) {
        super(map, logger);
    }

    @Override
    public void                         read(JSONObject reader) {

        super.read(reader);

        for (Map.Entry<String, org.firstinspires.ftc.core.subsystems.Subsystem> subsystem : mSubsystems.entrySet()) {
            if (subsystem.getKey().equals(sChassisKey)) {
                org.firstinspires.ftc.core.subsystems.Subsystem chassis = subsystem.getValue();
                if (chassis instanceof MecanumDrive) {
                    mChassis = (MecanumDrive) chassis;
                }
            }
        }

        if(mChassis != null) { mConfigurationValid = true; }
    }

    public void drive(double xSpeed, double ySpeed, double headingSpeed) {
        mChassis.drive(xSpeed, ySpeed, headingSpeed);
    }

    public void update() {
        mHardware.update();
        for (Map.Entry<String, Subsystem> subsystem : mSubsystems.entrySet()) {
            subsystem.getValue().update();
        }
    }
}
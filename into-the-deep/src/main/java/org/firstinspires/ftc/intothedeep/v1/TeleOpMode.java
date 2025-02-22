/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Into-The-Deep TeleOp mode
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/* ACME robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.intothedeep.v1.configuration.Configuration;

/* Components includes */
import org.firstinspires.ftc.core.components.controllers.Controller;

/* Robot includes */
import org.firstinspires.ftc.intothedeep.v1.robot.Robot;

/* Orchestration includes */
import org.firstinspires.ftc.intothedeep.v1.orchestration.Dispatcher;
import org.firstinspires.ftc.core.orchestration.engine.InterOpMode;

@TeleOp(name = "Robot V1 Teleop", group = "V1")
public class TeleOpMode extends LinearOpMode {

    LogManager              mLogger;

    Configuration           mConfiguration;

    Map<String, Controller> mControllers;
    Robot                   mRobot;
    Dispatcher              mDispatcher;
    
    @Override
    public void runOpMode() {

        try {



            // Log initialization
            mLogger = new LogManager(telemetry, FtcDashboard.getInstance(),"teleop-v1");
            mLogger.clear();

            InterOpMode.instance().clear();
            InterOpMode.instance().log(mLogger);

            // Configuration initialization
            mConfiguration = Configuration.getInstance();
            mConfiguration.logger(mLogger);

            // Robot initialization
            mRobot = new Robot(hardwareMap, mLogger);

            // Control initialization
            mControllers = new LinkedHashMap<>();
            mControllers.put("drive", new Controller(gamepad1, mLogger));
            mControllers.put("mechanisms", new Controller(gamepad2, mLogger));
            mDispatcher = new Dispatcher(mControllers, mRobot, mLogger);

            // Register configurables
            mConfiguration.register("logging", mLogger);
            mConfiguration.register("robot",mRobot);
            mConfiguration.register("control", mDispatcher);
            mConfiguration.read();
            mConfiguration.log();

        }
        catch(Exception e){
            mLogger.error(e.getMessage());
        }

        mLogger.update();

        waitForStart();


        mLogger.reset();
        // Starting position is not specified, then it will become
        // (0,0,0) if no data from previous OpMode, or the last OpMode pose if data from previous OpMode
        mRobot.start(Robot.Mode.TELEOP,null);

        while(opModeIsActive()) {

            try {

                mLogger.info(LogManager.Target.FILE, "loop start");
                mRobot.update();
                mRobot.log();
                mDispatcher.update();
                mLogger.info(LogManager.Target.FILE, "loop end");

            } catch (Exception e) {
                mLogger.error(e.getMessage());
            }

            mLogger.update();

            // Persist data if opmode is not stopped, or else motor may have reset
            if(!isStopRequested()) { mRobot.persist(); }
        }
        

        InterOpMode.instance().log(mLogger);
        mLogger.stop();
    }

}

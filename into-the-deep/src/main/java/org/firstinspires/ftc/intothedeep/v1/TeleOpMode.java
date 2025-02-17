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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* ACME robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.intothedeep.v1.configuration.Configuration;

/* Components includes */
import org.firstinspires.ftc.core.components.controllers.Controller;

@TeleOp(name = "Robot V1 Teleop", group = "V1")
public class TeleOpMode extends OpMode {

    LogManager      mLogger;

    Configuration   mConfiguration;

    Map<String, Controller> mControllers;
    
    @Override
    public void init(){

        try {

            // Log initialization
            mLogger = new LogManager(telemetry, FtcDashboard.getInstance(),"teleop-v1");
            mLogger.clear();

            // Configuration initialization
            mConfiguration = Configuration.getInstance();
            mConfiguration.logger(mLogger);

            // Robot initialization
            //mRobot = new SeasonRobot(mLogger);

            // Control initialization
//            mControllers = new LinkedHashMap<>();
//            mControllers.put("drive", new Controller(gamepad1, mLogger));
//            mControllers.put("mechanisms", new Controller(gamepad2, mLogger));
//            mControl = new ControlMapper(mControllers, mConfiguration, mLogger);
//
//            // Register configurables
//            mConfiguration.register("control", mControl);
//            mConfiguration.register("logging",mLogger);
            mConfiguration.read();
            mConfiguration.log();

            mLogger.update();

        }
        catch(Exception e){
            mLogger.error(e.getMessage());
        }

    }
    @Override
    public void loop (){

        try {

            mLogger.info("" + this.time);
            mLogger.update();
            

        }
        catch(Exception e){
            mLogger.error(e.getMessage());
        }
        
    }

    @Override
    public void stop() {
        mLogger.stop();
    }

}

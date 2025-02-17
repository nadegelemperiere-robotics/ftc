/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Into-The-Deep Autonomous Specimen Omode
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/* ACME robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.intothedeep.v1.configuration.Configuration;


@Autonomous(name = "Robot V1 Specimen Autonomous", group = "V1", preselectTeleOp = "Robot V1 Teleop")
public class AutonomousSpecimenOpMode extends LinearOpMode {

    LogManager      mLogger;

    Configuration   mConfiguration;

    @Override
    public void runOpMode(){

        try {
            // Log initialization
            mLogger = new LogManager(telemetry, FtcDashboard.getInstance(),"auto-specimen");

            // Configuration initialization
            mConfiguration = Configuration.getInstance();
            mConfiguration.logger(mLogger);

            // Register configurables
            mConfiguration.register("logging",mLogger);
            mConfiguration.read();
            mConfiguration.log();

            mLogger.update();
        }
        catch(Exception e){
            mLogger.error(e.getMessage());
        }

        waitForStart();



        try {

            if (!opModeIsActive()) return;

            mLogger.clear();
            mLogger.error("Error1");
            mLogger.warning("Warning1");
            mLogger.metric("COUNT","" + 1);
            mLogger.update();

            if (!opModeIsActive()) return;

            mLogger.clear();
            mLogger.error("Error2");
            mLogger.warning("Warning2");
            mLogger.metric("COUNT","" + 2);
            mLogger.update();

            if (!opModeIsActive()) return;

            mLogger.clear();
            mLogger.error("Error3");
            mLogger.warning("Warning3");
            mLogger.metric("COUNT","" + 3);
            mLogger.update();

            mLogger.stop();

        }
        catch(Exception e){
            mLogger.error(e.getMessage());
        }

    }
}

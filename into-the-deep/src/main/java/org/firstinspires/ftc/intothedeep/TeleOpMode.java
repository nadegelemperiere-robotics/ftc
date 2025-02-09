/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Into-The-Deep TeleOp mode
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* ACME robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

@TeleOp
public class TeleOpMode extends OpMode {

    LogManager          mLogger;
    
    @Override
    public void init(){

        try {

            // Log initialization
            mLogger = new LogManager(telemetry, FtcDashboard.getInstance(),"teleop");

        }
        catch(Exception e){

        }

    }
    @Override
    public void loop (){

        try {

            mLogger.clear();
            mLogger.error("error");
            mLogger.update();
            

        }
        catch(Exception e){

        }
        
    }

}

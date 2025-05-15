/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Into-The-Deep TeleOp mode
   ------------------------------------------------------- */

package tools;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Acmerobotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/* Tools includes */
import org.firstinspires.ftc.core.tools.Calibration;
import org.firstinspires.ftc.core.tools.LogManager;

@Config
@TeleOp(name = "CalibrationTuning", group = "V1")
public class CalibrationTuning extends LinearOpMode {


    static public double    X   = 0;
    static public double    Y   = 0;

    /* ---------------- Members ---------------- */
    LogManager      mLogger;
    Calibration     mCalibration;
    
    @Override
    public void runOpMode() {

        try {

            mLogger = new LogManager(null,FtcDashboard.getInstance(),"calibration-tuning");
            mLogger.level(LogManager.Severity.TRACE);

            mCalibration = new Calibration();
            mCalibration.initialize();

            String description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                    "<p style=\"font-weight: bold; font-size: 14px\">Enter a pixel coordinate in upper left corner reference. </p>" +
                    "<p style=\"font-weight: bold; font-size: 14px\">The tool will compute the ground distance in inches of the object seen on this pixel</p>" +
                    "<p style=\"font-weight: bold; font-size: 14px\">The X axes is parallel to the camera, oriented right. </p>" +
                    "<p style=\"font-weight: bold; font-size: 14px\">The Y axes is orthogonal to the camera, oriented away. </p>";
            mLogger.info(LogManager.Target.DASHBOARD,description);

        }
        catch(Exception e){
            telemetry.addLine(e.getMessage());
        }

        waitForStart();

        while(opModeIsActive()) {

            float [] result = mCalibration.computeGroundPosition(X,Y);

            FtcDashboard.getInstance().getTelemetry().addData("x (inches)",result[0]);
            FtcDashboard.getInstance().getTelemetry().addData("y (inches)",result[1]);

            FtcDashboard.getInstance().getTelemetry().update();
        }
    }
}
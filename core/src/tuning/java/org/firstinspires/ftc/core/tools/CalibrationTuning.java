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

@Config
@TeleOp(name = "CalibrationTuning", group = "V1")
public class CalibrationTuning extends LinearOpMode {


    static public double    X   = 0;
    static public double    Y   = 0;
    Calibration mCalibration;
    
    @Override
    public void runOpMode() {

        try {

            mCalibration = new Calibration();
            mCalibration.initialize();

        }
        catch(Exception e){
            telemetry.addLine(e.getMessage());
        }

        waitForStart();

        while(opModeIsActive()) {

            float [] result = mCalibration.computeGroundPosition(X,Y);

            FtcDashboard.getInstance().getTelemetry().addData("x",result[0]);
            FtcDashboard.getInstance().getTelemetry().addData("y",result[1]);

            FtcDashboard.getInstance().getTelemetry().update();
        }
    }
}
/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Sample detection test tool
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1;

/* Android includes */
import android.graphics.Bitmap;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* ACME robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/* OpenCV includes */
import org.opencv.android.Utils;
import org.opencv.core.Mat;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Components includes */
import org.firstinspires.ftc.core.components.cameras.CameraComponent;

/* Processing includes */
import org.firstinspires.ftc.intothedeep.v1.processing.SamplesDetection;
import org.firstinspires.ftc.intothedeep.v1.processing.Sample;

/* Tuning includes */
import org.firstinspires.ftc.core.tuning.Tuning;
import org.firstinspires.ftc.intothedeep.v1.tuning.Robot;

@Config
@TeleOp(name = "SamplesDetectionTest", group = "Test")
public class SamplesDetectionTest extends LinearOpMode  implements Tuning {

    static public Sample.Color  COLOR       = Sample.Color.YELLOW;
    public static String        DETECTOR    = "sample-detection";
    public static String        CAMERA      = "limelight";

    /* ---------------- Members ---------------- */
    private LogManager          mLogger;

    private Configuration       mConfiguration;

    private CameraComponent     mCamera;
    private Robot               mRobot;
    private SamplesDetection    mDetection;

    @Override
    public void runOpMode() {

        try {

            mLogger = new LogManager(null,FtcDashboard.getInstance(),"samples-detection-test");
            mLogger.level(LogManager.Severity.TRACE);

            mRobot = new Robot(this, hardwareMap, mLogger);

            mConfiguration = new Configuration(mLogger);
            mConfiguration.register("robot", mRobot);
            mConfiguration.read();
            mConfiguration.log();

            mCamera = null;
            mDetection = null;
            if(mRobot.isConfigured()) {
                mCamera = mRobot.hardware(this).cameras().get(CAMERA);
                mDetection = (SamplesDetection) mRobot.processor(this, DETECTOR);
            }
            if(mDetection != null) { mDetection.start(); }

            FtcDashboard.getInstance().updateConfig();
            mLogger.update();

            waitForStart();

            mLogger.clear();

            while(opModeIsActive()) {

                mDetection.color(COLOR);
                mRobot.update();
                mDetection.log("");

                Mat frame = mCamera.current();
                if (frame != null) {
                    Mat overlays = mDetection.draw(frame);
                    Bitmap bitmap = Bitmap.createBitmap(overlays.cols(), overlays.rows(), Bitmap.Config.ARGB_8888);
                    Utils.matToBitmap(overlays, bitmap);
                    FtcDashboard.getInstance().sendImage(bitmap);
                }
                else {
                    mLogger.warning("Received null frame");
                }

                // Log cameras state and updated configuration
                mLogger.update();

            }

        }
        catch(Exception e) {
            FtcDashboard.getInstance().getTelemetry().addLine(e.toString());
            FtcDashboard.getInstance().getTelemetry().update();
        }

        mLogger.stop();
    }
}
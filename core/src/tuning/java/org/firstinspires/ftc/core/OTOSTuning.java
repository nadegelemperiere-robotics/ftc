/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   OTOS tuning tool
   ------------------------------------------------------- */

package org.firstinspires.ftc.core;

/* System includes */
import java.security.InvalidParameterException;
import java.util.Map;

/* Android includes */
import android.os.Environment;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* ACME robotics includes */
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Components includes */
import org.firstinspires.ftc.core.components.odometers.OdometerComponent;
import org.firstinspires.ftc.core.components.odometers.OpticalTrackingOdometer;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Tuning;

@Config
@TeleOp(name = "OTOSTuning", group = "Tuning")
public class OTOSTuning extends LinearOpMode {

    public enum Param {
        HEADING_RATIO,
        HEADING_OFFSET,
        POSITION_RATIO,
        POSITION_OFFSET
    }

    public enum Step {
        STOP,
        INIT,
        PROCESSING,
        UPDATE
    }

    /* -------- Configuration variables -------- */
    public static Step                  STEP    = Step.STOP;
    public static Param                 TUNING  = Param.HEADING_RATIO;
    public static String                CONFIGURATION   = "test";

    /* ---------------- Members ---------------- */
    private LogManager                  mLogger;
    private Param                       mPreviousTuning;

    private Configuration               mConfiguration;
    private String                      mConfigurationName;
    private Tuning                      mHardware;

    private OpticalTrackingOdometer     mOdometer;

    private double                      mHeadingRatioRadTurned;
    private Rotation2d                  mHeadingRatioLastHeading;
    private DistanceProvider            mPositionRatioDistance;

    @Override
    public void runOpMode() {

        try {

            mLogger = new LogManager(null,FtcDashboard.getInstance(),"otos-tuning");
            mLogger.level(LogManager.Severity.TRACE);

            mHardware = new Tuning(hardwareMap, mLogger);

            mPositionRatioDistance  = new DistanceProvider();


            mPreviousTuning = TUNING;

            mConfigurationName = CONFIGURATION;
            mConfiguration = new Configuration(mLogger);
            mConfiguration.register("robot.hardware", mHardware);
            mConfiguration.read(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/" + mConfigurationName + ".json");
            mConfiguration.log();

            Map<String,OdometerComponent>    odometers = mHardware.odometers();

            mOdometer = null;
            if(odometers.containsKey("otos")) {
                mOdometer = ((OpticalTrackingOdometer) odometers.get("otos"));
            }
            if(mOdometer == null) throw new InvalidParameterException("Not otos based odometer found in configuration");

            FtcDashboard.getInstance().updateConfig();
            mLogger.update();

            waitForStart();

            mLogger.clear();

            while(opModeIsActive()) {

                // Manage tuning change
                if(mPreviousTuning != TUNING) {
                    STEP = Step.STOP;
                    mPreviousTuning = TUNING;
                }

                // Manage step change
                String description;
                switch (TUNING) {
                    case HEADING_RATIO:
                        description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                                "<p style=\"font-weight: bold; font-size: 14px\"> OTOS HEADING RATIO TUNER </p>" +
                                "<p style=\"font-size: 12px\"> Switch to PROCESSING and rotate the robot on the ground 10 times. </p>" +
                                "<p style=\"font-size: 12px\"> Make sure you mark the starting orientation precisely. </p>"+
                                "<p style=\"font-size: 12px\"> Make sure you fit exactly to this orientation at the end of your test. </p>" +
                                "<p style=\"font-size: 12px\"> Finally step to the UPDATE step to update the configuration with the value. </p>";
                        mLogger.info(LogManager.Target.DASHBOARD,description);
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"DISTANCE");
                        this.processHeadingRatio(); break;
                    case HEADING_OFFSET:
                        description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                                "<p style=\"font-weight: bold; font-size: 14px\"> OTOS HEADING OFFSET TUNER </p>" +
                                "<p style=\"font-size: 12px\"> Line the side of the robot against a wall and switch to PROCESSING </p>" +
                                "<p style=\"font-size: 12px\"> Then push the robot forward some distance. </p>"+
                                "<p style=\"font-size: 12px\"> Finally step to the UPDATE step to update the configuration with the value. </p>";
                        mLogger.info(LogManager.Target.DASHBOARD,description);
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"DISTANCE");
                        this.processHeadingOffset(); break;
                    case POSITION_RATIO:
                        description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                                "<p style=\"font-weight: bold; font-size: 14px\"> OTOS POSITION RATIO TUNER </p>" +
                                "<p style=\"font-size: 12px\"> Set the distance your about to move in the Tuning parameters and switch to PROCESSING </p>" +
                                "<p style=\"font-size: 12px\"> Then push the robot forward this same distance (make sure you measure it precisely). </p>" +
                                "<p style=\"font-size: 12px\"> Finally, step to the UPDATE step to update the configuration with the value. </p>";
                        mLogger.info(LogManager.Target.DASHBOARD,description);
                        FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"DISTANCE",mPositionRatioDistance);
                        this.processPositionRatio(); break;
                    case POSITION_OFFSET:
                        description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                                "<p style=\"font-weight: bold; font-size: 14px\"> OTOS POSITION OFFSET TUNER </p>" +
                                "<p style=\"font-size: 12px\"> Line the robot against the corner of two walls facing forward and switch to PROCESSING </p>" +
                                "<p style=\"font-size: 12px\"> Then rotate the robot exactly 180 degrees and press it back into the corner. </p>" +
                                "<p style=\"font-size: 12px\"> Finally, step to the UPDATE step to update the configuration with the value. </p>";
                        mLogger.info(LogManager.Target.DASHBOARD,description);
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"DISTANCE");
                        this.processPositionOffset(); break;
                }


                mOdometer.update();
                mOdometer.log();

                mConfiguration.log();

                mLogger.update();
            }

           mConfiguration.write(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/otos-tuning.json");
            mLogger.info("Updated configuration saved. You may retrieve it using <b>adb pull /sdcard/FIRST/otos-tuning.json</b>");
            mLogger.update();
        }
        catch(Exception e) {
            mLogger.error(e.toString());
            mLogger.update();
        }
    }

    void processHeadingRatio() {

        switch(STEP) {
            case INIT:
                mHeadingRatioRadTurned = 0;
                mHeadingRatioLastHeading = Rotation2d.fromDouble(0);
                mOdometer.pose(new Pose2d(new Vector2d(0, 0), 0));

                break;

            case PROCESSING:
                Pose2d pose = mOdometer.pose();
                mHeadingRatioRadTurned += pose.heading.minus(mHeadingRatioLastHeading);
                mHeadingRatioLastHeading = pose.heading;
                mLogger.info("Uncorrected Degrees Turned " +  Math.round(Math.toDegrees(mHeadingRatioRadTurned)));
                mLogger.info("Calculated Heading Ratio " + 3600 / Math.toDegrees(mHeadingRatioRadTurned));
                break;

            case UPDATE :
                mLogger.info("Uncorrected Degrees Turned " +  Math.round(Math.toDegrees(mHeadingRatioRadTurned)));
                mLogger.info("Calculated Heading Ratio " + 3600 / Math.toDegrees(mHeadingRatioRadTurned));
                mOdometer.headingRatio(3600 / Math.toDegrees(mHeadingRatioRadTurned));
                break;
        }

    }

    void processHeadingOffset() {

        Pose2d pose;
        double offset;

        switch(STEP) {
            case INIT :
                mOdometer.pose(new Pose2d(new Vector2d(0,0),0));
                break;
            case PROCESSING :
                pose = mOdometer.pose();
                offset = Math.atan2(pose.position.y,pose.position.x);
                mLogger.info("Heading Offset (radians, enter this one into SparkFunOTOSDrive!) " + offset);
                mLogger.info("Heading Offset (degrees) "+ Math.toDegrees(offset));
                break;
            case UPDATE :
                pose = mOdometer.pose();
                offset = Math.atan2(pose.position.y,pose.position.x);
                mLogger.info("Heading Offset (radians, enter this one into SparkFunOTOSDrive!) " + offset);
                mLogger.info("Heading Offset (degrees) "+ Math.toDegrees(offset));
                mOdometer.headingOffset(Math.atan2(pose.position.y,pose.position.x));
                break;
        }

    }

    void processPositionRatio() {

        Pose2d pose;
        double distance;

        switch(STEP) {
            case INIT:

                mOdometer.pose(new Pose2d(new Vector2d(0, 0), 0));
                break;
            case PROCESSING:
                pose = mOdometer.pose();
                distance = Math.sqrt(pose.position.x * pose.position.x + pose.position.y * pose.position.y);
                mLogger.info("Uncorrected Inches Moved " + distance);
                mLogger.info("Calculated Position Ratio " + mPositionRatioDistance.get() / distance);
                break;
            case UPDATE:
                pose = mOdometer.pose();
                distance = Math.sqrt(pose.position.x * pose.position.x + pose.position.y * pose.position.y);
                mLogger.info("Uncorrected Inches Moved " + distance);
                mLogger.info("Calculated Position Ratio " + mPositionRatioDistance.get() / distance);
                mOdometer.positionRatio(mPositionRatioDistance.get() / distance);
                break;
        }

    }

    void processPositionOffset() {

        Pose2d pose;

        switch(STEP) {
            case STOP:
                break;
            case INIT:
                mOdometer.pose(new Pose2d(new Vector2d(0, 0), 0));
                break;
            case PROCESSING:
                pose = mOdometer.pose();
                if (Math.abs(Math.toDegrees(pose.heading.toDouble())) > 175) {
                    mLogger.info("X Offset " + -0.5 * pose.position.x);
                    mLogger.info("Y Offset " + -0.5 * pose.position.y);
                }
                else {
                    mLogger.info( LogManager.Target.DASHBOARD,"<p style=\"font-size: 12px\">Rotate the robot 180 degrees and align it to the corner again.</p>");
                }
                break;
            case UPDATE:
                pose = mOdometer.pose();
                mLogger.info("X Offset " + -0.5 * pose.position.x);
                mLogger.info("Y Offset " + -0.5 * pose.position.y);
                mOdometer.xOffset(-0.5 * pose.position.x);
                mOdometer.yOffset( -0.5 * pose.position.y);
                break;
        }

    }

    // Since double is a simple type, even with the appropriate constructor,
    // mDistance will only be updated locally by the dashboard.
    // We'll have to make sure the code access the mode from the provider using the get
    // Method, since it's the only place the updated information can be found
    static class DistanceProvider implements ValueProvider<Double> {
        double mDistance;
        @Override
        public Double   get()                { return mDistance;  }
        @Override
        public void     set(Double Value)    { mDistance = Value; }
    }
}
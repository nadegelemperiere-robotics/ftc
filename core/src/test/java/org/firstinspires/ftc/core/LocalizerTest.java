/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localizers lateral tuning tool
   ------------------------------------------------------- */

package org.firstinspires.ftc.core;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

/* Android includes */
import android.os.Environment;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* ACME robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;

/* PedroPathing includes */
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.DashboardPoseTracker;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Components includes */
import org.firstinspires.ftc.core.components.controllers.Controller;
import org.firstinspires.ftc.core.components.motors.MotorComponent;
import org.firstinspires.ftc.core.components.localizers.LocalizerComponent;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

/* Tuning includes */
import org.firstinspires.ftc.core.tuning.Tuning;

@Config
@TeleOp(name = "LocalizerTest", group = "Tuning")
public class LocalizerTest extends LinearOpMode implements Tuning {

    /* -------- Configuration variables -------- */
    public static String                        CONFIGURATION   = "test";
    public static String                        LEFT_BACK       = "back-left-wheel";
    public static String                        LEFT_FRONT      = "front-left-wheel";
    public static String                        RIGHT_BACK      = "back-right-wheel";
    public static String                        RIGHT_FRONT     = "front-right-wheel";

    /* ---------------- Members ---------------- */
    private LogManager                          mLogger;

    private Configuration                       mConfiguration;
    private String                              mConfigurationName;
    private Hardware                            mHardware;

    private Controller                          mController;

    /* ---- Preload for all localizers data  --- */

    // The localizer selection config variables that can be updated by the dashboard
    private Map<String, Boolean>                mLocalizerSelection;
    private String                              mCurrentLocalizer;
    private MotorComponent                      mLeftBack;
    private MotorComponent                      mLeftFront;
    private MotorComponent                      mRightBack;
    private MotorComponent                      mRightFront;

    private LocalizerComponent                  mSelectedLocalizer;
    private PoseUpdater                         mSelectedUpdater;
    private DashboardPoseTracker                mDashboardPoseTracker;

    // Link between name and the corresponding localizer
    private Map<String, LocalizerComponent>     mLocalizers;



    @Override
    public void runOpMode() {

        try {

            mLogger = new LogManager(null,FtcDashboard.getInstance(),"localization-test-tuning");
            mLogger.level(LogManager.Severity.TRACE);

            mController = new Controller(gamepad1, mLogger);

            mHardware = new Hardware(hardwareMap, mLogger);

            mConfigurationName = CONFIGURATION;
            mConfiguration = new Configuration(mLogger);
            mConfiguration.register("robot.hardware", mHardware);
            mConfiguration.read(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/" + mConfigurationName + ".json");
            mConfiguration.log();

            mLocalizers = mHardware.localizers();
            mSelectedLocalizer = null;

            mLeftBack = null;
            mRightBack = null;
            mLeftFront = null;
            mRightFront = null;
            if(mHardware.motors().containsKey(LEFT_BACK))   { mLeftBack = mHardware.motors().get(LEFT_BACK);        }
            if(mHardware.motors().containsKey(RIGHT_BACK))  { mRightBack = mHardware.motors().get(RIGHT_BACK);      }
            if(mHardware.motors().containsKey(LEFT_FRONT))  { mLeftFront = mHardware.motors().get(LEFT_FRONT);      }
            if(mHardware.motors().containsKey(RIGHT_FRONT)) { mRightFront = mHardware.motors().get(RIGHT_FRONT);    }
            if(mLeftBack == null) { mLogger.error("Missing left back motor with name " + LEFT_BACK); }
            if(mRightBack == null) { mLogger.error("Missing right back motor with name " + RIGHT_BACK); }
            if(mLeftFront == null) { mLogger.error("Missing left front motor with name " + LEFT_FRONT); }
            if(mRightFront == null) { mLogger.error("Missing right front motor with name " + RIGHT_FRONT); }

            mLocalizerSelection = new LinkedHashMap<>();
            for (Map.Entry<String, LocalizerComponent> localizer : mLocalizers.entrySet()) {
                mLocalizerSelection.put(localizer.getKey(),false);
            }

            // Add the localizer selection variables on the dashboard
            for (Map.Entry<String, Boolean> selected : mLocalizerSelection.entrySet()) {
                SelectedProvider provider = new SelectedProvider(mLocalizerSelection, selected.getKey());
                FtcDashboard.getInstance().addConfigVariable(LocalizerTest.class.getSimpleName(),selected.getKey(),provider);
            }

            String description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                    "<p style=\"font-weight: bold; font-size: 14px\">This will print your robot's position to telemetry while allowing robot control through a basic mecanum drive on gamepad 1.</p>";
            mLogger.info(LogManager.Target.DASHBOARD,description);

            FtcDashboard.getInstance().updateConfig();
            mLogger.update();

            waitForStart();

            mLogger.clear();

            while(opModeIsActive()) {

                /* Find current selected motor */
                String currentLocalizer = this.findSelectedLocalizer();

                /* Manage configuration change */
                if(!Objects.equals(currentLocalizer, mCurrentLocalizer))  {

                    // Now we can forget the previously selected motors sice we no longer need them
                    mCurrentLocalizer = currentLocalizer;
                    // Select hardware motors and conf for current motor
                    this.updateCurrentLocalizer();

                }

                /* Manage motor change */
                if(!Objects.equals(mLeftBack.name(), LEFT_BACK)) {
                    if(mHardware.motors().containsKey(LEFT_BACK))   { mLeftBack = mHardware.motors().get(LEFT_BACK);        }
                    if(mLeftBack == null) { mLogger.error("Missing left back motor with name " + LEFT_BACK); }
                }
                if(!Objects.equals(mLeftFront.name(), LEFT_FRONT)) {
                    if(mHardware.motors().containsKey(LEFT_FRONT))   { mLeftFront = mHardware.motors().get(LEFT_FRONT);        }
                    if(mLeftFront == null) { mLogger.error("Missing left front motor with name " + LEFT_FRONT); }
                }
                if(!Objects.equals(mRightBack.name(), RIGHT_BACK)) {
                    if(mHardware.motors().containsKey(RIGHT_BACK))   { mRightBack = mHardware.motors().get(RIGHT_BACK);        }
                    if(mRightBack == null) { mLogger.error("Missing right back motor with name " + RIGHT_BACK); }
                }
                if(!Objects.equals(mRightFront.name(), RIGHT_FRONT)) {
                    if(mHardware.motors().containsKey(RIGHT_FRONT))   { mRightFront = mHardware.motors().get(RIGHT_FRONT);        }
                    if(mRightFront == null) { mLogger.error("Missing right front motor with name " + RIGHT_FRONT); }
                }

                double y = mController.axes.left_stick_y.value();
                double x = mController.axes.left_stick_x.value();
                double rx = mController.axes.right_stick_x.value();

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double leftFrontPower = (y + x + rx) / denominator;
                double leftRearPower = (y - x + rx) / denominator;
                double rightFrontPower = (y - x - rx) / denominator;
                double rightRearPower = (y + x - rx) / denominator;

                if(mLeftFront != null)  { mLeftFront.power(leftFrontPower); }
                if(mRightFront != null) { mRightFront.power(rightFrontPower); }
                if(mLeftBack != null)   { mLeftBack.power(leftRearPower); }
                if(mRightBack != null)  { mRightBack.power(rightRearPower); }

                if(mSelectedLocalizer != null)      { mSelectedLocalizer.update(); }
                if(mDashboardPoseTracker != null)   {mSelectedUpdater.update(); }
                if(mDashboardPoseTracker != null)   { mDashboardPoseTracker.update(); }

                // Log motors state and updated configuration
                this.logLocalizerState(mLogger);
                mConfiguration.log();

                mLogger.metric("Current Localizer",currentLocalizer);

                if(mDashboardPoseTracker != null)   {Drawing.drawPoseHistory(mDashboardPoseTracker, "#4CAF50"); }
                if(mDashboardPoseTracker != null)   {Drawing.drawRobot(mSelectedUpdater.getPose(), "#4CAF50"); }
                Drawing.sendPacket();

                mLogger.update();
            }

            mLogger.update();
        }
        catch(Exception e) {
            mLogger.error(e.toString());
            mLogger.update();
        }
    }

    private String findSelectedLocalizer()
    {
        String result = "";
        for (Map.Entry<String, Boolean> selected : mLocalizerSelection.entrySet()) {
            if(selected.getValue()) { result = selected.getKey(); }
        }
        return result;
    }

    private void   updateCurrentLocalizer() {

        mSelectedLocalizer = null;

        if (mLocalizers.containsKey(mCurrentLocalizer)) {
            mSelectedLocalizer = mLocalizers.get(mCurrentLocalizer);
            mSelectedUpdater = new PoseUpdater(null, mSelectedLocalizer);
            mDashboardPoseTracker = new DashboardPoseTracker(mSelectedUpdater);

            mSelectedLocalizer.setStartPose(new Pose(0,0,0));
            mSelectedLocalizer.setPose(new Pose(0,0,0));

        }
    }

    private void logLocalizerState(LogManager logger) {

        logger.info("CURRENT LOCALIZER");

        if (mSelectedLocalizer != null) {

            mSelectedLocalizer.log();

            double heading = mSelectedUpdater.getPose().getHeading();
            if(heading > Math.PI) { heading -= 2 * Math.PI; }

            logger.info("-----> x : " +  mSelectedUpdater.getPose().getX());
            logger.info("-----> y : " +  mSelectedUpdater.getPose().getY());
            logger.info("-----> heading : " +  heading);
            logger.info("-----> total heading : " +  mSelectedUpdater.getTotalHeading());

        }
    }

    // SelectedProvider updates the motors selection states
    // Since Map<String, Boolean> is not a simple type, it's managed as
    // pointer, when we change it in the provider, it's changed in the
    // global class.
    // When we select a new motor, we make sure to deselect all the others
    static class SelectedProvider implements ValueProvider<Boolean> {
        Map<String, Boolean> mAllSelection;
        String mCurrentSelection;

        public SelectedProvider(Map<String, Boolean> selection, String current) {
            mAllSelection = selection;
            mCurrentSelection = current;
        }

        @Override
        public Boolean get() {
            return mAllSelection.get(mCurrentSelection);
        }

        @Override
        public void set(Boolean Value) {

            if (Value) {
                for (Map.Entry<String, Boolean> selected : mAllSelection.entrySet()) {
                    selected.setValue(false);
                }
            }
            mAllSelection.put(mCurrentSelection, Value);
        }
    }


}
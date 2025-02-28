/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localizers lateral tuning tool
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.localizers;

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

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

/* Tuning includes */
import org.firstinspires.ftc.core.tuning.Tuning;

@Config
@TeleOp(name = "LateralTuning", group = "Tuning")
public class LateralTuning extends LinearOpMode implements Tuning {

    /* -------- Configuration variables -------- */
    public static String                        CONFIGURATION   = "test";
    public static double                        DISTANCE        = 48;

    /* ---------------- Members ---------------- */
    private LogManager                          mLogger;

    private Configuration                       mConfiguration;
    private String                              mConfigurationName;
    private Hardware                            mHardware;

    private SaveProvider                        mShallSave;
    private double                              mMultiplier;

    /* ---- Preload for all localizers data  --- */

    // The localizer selection config variables that can be updated by the dashboard
    private Map<String, Boolean>                mLocalizerSelection;
    private String                              mCurrentLocalizer;
    private LocalizerComponent                  mSelectedLocalizer;
    private PoseUpdater                         mSelectedUpdater;
    private DashboardPoseTracker                mDashboardPoseTracker;

    // Link between name and the corresponding localizer
    private Map<String, LocalizerComponent>     mLocalizers;



    @Override
    public void runOpMode() {

        try {

            mLogger = new LogManager(null,FtcDashboard.getInstance(),"localizer-lateral-tuning");
            mLogger.level(LogManager.Severity.TRACE);

            mHardware = new Hardware(hardwareMap, mLogger);

            mConfigurationName = CONFIGURATION;
            mConfiguration = new Configuration(mLogger);
            mConfiguration.register("robot.hardware", mHardware);
            mConfiguration.read(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/" + mConfigurationName + ".json");
            mConfiguration.log();

            mLocalizers = mHardware.localizers();

            mLocalizerSelection = new LinkedHashMap<>();
            for (Map.Entry<String, LocalizerComponent> localizer : mLocalizers.entrySet()) {
                mLocalizerSelection.put(localizer.getKey(),false);
            }

            // Add the localizer selection variables on the dashboard
            for (Map.Entry<String, Boolean> selected : mLocalizerSelection.entrySet()) {
                SelectedProvider provider = new SelectedProvider(mLocalizerSelection, selected.getKey());
                FtcDashboard.getInstance().addConfigVariable(LateralTuning.class.getSimpleName(),selected.getKey(),provider);
            }
            mShallSave = new SaveProvider();
            FtcDashboard.getInstance().addConfigVariable(LateralTuning.class.getSimpleName(),"SAVE",mShallSave);

            String description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                    "<p style=\"font-weight: bold; font-size: 14px\"> Pull your robot laterally " + DISTANCE + " inches. Your lateral ticks to inches will be shown on the telemetry. </p>";
            mLogger.info(LogManager.Target.DASHBOARD,description);

            FtcDashboard.getInstance().updateConfig();
            mLogger.update();

            waitForStart();

            mLogger.clear();

            while(opModeIsActive()) {

                description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                        "<p style=\"font-weight: bold; font-size: 14px\"> LATERAL TUNING </p>" +
                        "<p style=\"font-size: 12px\"> The multiplier will display what your lateral multiplies should be to scale your current distance to " + DISTANCE + " inches. </p>" ;
                mLogger.info(LogManager.Target.DASHBOARD,description);

                /* Find current selected motor */
                String currentLocalizer = this.findSelectedLocalizer();

                /* Manage configuration change */
                if(!Objects.equals(currentLocalizer, mCurrentLocalizer))  {

                    // Save current localizer
                    if(mShallSave.get()) { mSelectedLocalizer.setLateralMultiplier(mMultiplier); }

                    // Now we can forget the previously selected motors sice we no longer need them
                    mCurrentLocalizer = currentLocalizer;
                    // Select hardware motors and conf for current motor
                    this.updateCurrentLocalizer();

                }

                mSelectedLocalizer.update();
                mSelectedUpdater.update();
                mDashboardPoseTracker.update();
                mMultiplier = DISTANCE / (mSelectedUpdater.getPose().getY() / mSelectedUpdater.getLocalizer().getLateralMultiplier());

                // Log motors state and updated configuration
                this.logLocalizerState(mLogger);
                mConfiguration.log();

                mLogger.metric("Current Localizer",currentLocalizer);
                mLogger.metric("Multiplier","" + mMultiplier);

                Drawing.drawPoseHistory(mDashboardPoseTracker, "#4CAF50");
                Drawing.drawRobot(mSelectedUpdater.getPose(), "#4CAF50");
                Drawing.sendPacket();

                mLogger.update();
            }

            mConfiguration.write(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/localization-lateral-tuning.json");
            mLogger.info("Updated configuration saved. You may retrieve it using <b>adb pull /sdcard/FIRST/localization-lateral-tuning.json</b>");
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

            mShallSave.set(false);

        }
    }

    private void logLocalizerState(LogManager logger) {
        logger.info("CURRENT LOCALIZER");

        if (mSelectedLocalizer != null) {

            // Log localizer state
            logger.info("-----> HwMap : " + mSelectedLocalizer.name());
            logger.info("-----> Pose : " + mSelectedLocalizer.getPose());
            logger.info("-----> Velocity : " + mSelectedLocalizer.getVelocity());
            logger.info("-----> Multiplier : " + mSelectedLocalizer.getLateralMultiplier());

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


    // SaveProvider updates the controller reverse configuration
    // Since ConfMotor.Controller is not a simple type, it's managed as
    // pointer, when we change it in the provider, it's changed in the
    // global configuration
    static class SaveProvider implements ValueProvider<Boolean> {
        boolean mShallSave;
        public SaveProvider( ) {
            mShallSave = false;
        }
        @Override
        public Boolean get()           { return mShallSave; }
        @Override
        public void set(Boolean value) { mShallSave = value;   }
    }

}
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
import org.firstinspires.ftc.core.components.odometers.DriveEncodersOdometer;
import org.firstinspires.ftc.core.components.imus.ImuBuiltIn;
import org.firstinspires.ftc.core.components.imus.ImuComponent;

/* Subsystem includes */
import org.firstinspires.ftc.core.subsystems.DriveTrain;
import org.firstinspires.ftc.core.subsystems.Subsystem;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Tuning;

/* Tuning includes */
import org.firstinspires.ftc.core.tuning.InPerTick;
import org.firstinspires.ftc.core.tuning.TrackWidthTicks;

@Config
@TeleOp(name = "DriveEncoderTuning", group = "Tuning")
public class DriveEncoderTuning extends LinearOpMode {

    public enum Param {
        FWD_IN_PER_TICK,
        LAT_IN_PER_TICK,
        TRACK_WIDTH_TICKS
    }

    public enum Step {
        STOP,
        INIT,
        PROCESSING,
        UPDATE
    }

    /* -------- Configuration variables -------- */
    public static Step                  STEP    = Step.STOP;
    public static Param                 TUNING  = Param.FWD_IN_PER_TICK;
    public static String                CONFIGURATION   = "test";

    /* ---------------- Members ---------------- */
    private LogManager                  mLogger;
    private Param                       mPreviousTuning;

    private Configuration               mConfiguration;
    private String                      mConfigurationName;
    private Tuning                      mHardware;

    private DriveEncodersOdometer       mOdometer;
    private DriveTrain                  mDriveTrain;
    private ImuBuiltIn                  mImu;

    private InPerTick                   mFwdInPerTick;
    private InPerTick                   mLatInPerTick;
    private TrackWidthTicks             mTrackWidthTicks;

    private DistanceProvider            mInPerTickDistance;
    private PowerProvider               mPowerPerSecond;
    private PowerProvider               mMaximalPower;
    private StartProvider               mShallStart;
    private TrackWidthProvider          mTrackWidth;

    private boolean                     mFirstTimeCalled;
    private boolean                     mFirstTimeStopped;
    private boolean                     mShallSave;

    @Override
    public void runOpMode() {

        try {

            mLogger = new LogManager(null,FtcDashboard.getInstance(),"drive-encoders-tuning");
            mLogger.level(LogManager.Severity.TRACE);

            mHardware = new Tuning(hardwareMap, mLogger);
            mDriveTrain = null;

            mPreviousTuning = TUNING;

            mConfigurationName = CONFIGURATION;
            mConfiguration = new Configuration(mLogger);
            mConfiguration.register("robot.hardware", mHardware);
            mConfiguration.read(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/" + mConfigurationName + ".json");
            mConfiguration.log();
            mDriveTrain = (DriveTrain)Subsystem.factory("drive-train",mConfiguration.search("robot.subsystems.drive-train"),mHardware,mLogger);

            Map<String,OdometerComponent>    odometers = mHardware.odometers();

            mOdometer = null;
            if(odometers.containsKey("driveencoders")) {
                mOdometer = ((DriveEncodersOdometer) odometers.get("driveencoders"));
            }
            if(mOdometer == null) throw new InvalidParameterException("No drive encoders based odometer found in configuration");

            mImu = null;
            Map<String,ImuComponent> imus = mHardware.imus();
            if(imus.containsKey(ImuComponent.sBuiltInKey)) { mImu = (ImuBuiltIn)imus.get(ImuComponent.sBuiltInKey); }
            if(mImu == null) throw new InvalidParameterException("No built in imu found in configuration");


            mInPerTickDistance  = new DistanceProvider();
            mPowerPerSecond     = new PowerProvider();
            mMaximalPower       = new PowerProvider();
            mShallStart         = new StartProvider();
            mTrackWidth         = new TrackWidthProvider(mOdometer);
            mMaximalPower.set(1.0);
            mPowerPerSecond.set(0.1);
            mInPerTickDistance.set(64.0);

            mFwdInPerTick = null;
            mLatInPerTick = null;
            mTrackWidthTicks = null;

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
                    case FWD_IN_PER_TICK:
                        description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                                "<p style=\"font-weight: bold; font-size: 14px\"> FORWARD INCHES PER TICK TUNER </p>" +
                                "<p style=\"font-size: 12px\"> Set the distance you'er about to move in the Tuning parameters and switch to PROCESSING </p>" +
                                "<p style=\"font-size: 12px\"> Then push the robot forward this same distance (make sure you measure it precisely). </p>" +
                                "<p style=\"font-size: 12px\"> Finally, step to the UPDATE step to update the configuration with the value. </p>";
                        mLogger.info(LogManager.Target.DASHBOARD,description);
                        FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"DISTANCE",mInPerTickDistance);
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"START");
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"POWER_PER_SECOND");
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"MAXIMAL_POWER");
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"TRACK_WIDTH");
                        this.processFwdInPerTick();
                        break;
                    case LAT_IN_PER_TICK:
                        description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                                "<p style=\"font-size: 12px\"> LATERAL INCHES PER TICK TUNER <span style=\"font-color:orange\">MECANUM DRIVE ONLY</span> </p>" +
                                "<p style=\"font-size: 12px\"> Set the distance you're about to move in the Tuning parameters and switch to PROCESSING </p>" +
                                "<p style=\"font-size: 12px\"> Then push the robot laterally this same distance (make sure you measure it precisely). </p>" +
                                "<p style=\"font-size: 12px\"> Finally, step to the UPDATE step to update the configuration with the value. </p>";
                        mLogger.info(LogManager.Target.DASHBOARD,description);
                        FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"DISTANCE",mInPerTickDistance);
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"START");
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"POWER_PER_SECOND");
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"MAXIMAL_POWER");
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"TRACK_WIDTH");
                        this.processLatInPerTick();
                        break;
                    case TRACK_WIDTH_TICKS :
                        description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                                "<p style=\"font-size: 12px\"> TRACK WIDTH TUNER </p>" +
                                "<p style=\"font-size: 12px\"> The robot will rotate on itself once you step into processing and trigger the sart button</p>" +
                                "<p style=\"font-size: 12px\"> Stop it at any point keeping in mind that longer runs will collect more data. </p>" +
                                "<p style=\"font-size: 12px\"> Finally, step to UPDATE : all data collected is saved to a file for further analysis.  </p>" +
                                "<p style=\"font-size: 12px\"> Go to http://192.168.43.1:8080/tuning/drive-encoder-angular-ramp.html and click the “Latest” button.  </p>" +
                                "<p style=\"font-size: 12px\"> Remove outliers from the estimate, get track width tick from the the “Ramp Regression” plot and update odometer configuration with it  </p>";
                        mLogger.info(LogManager.Target.DASHBOARD,description);
                        FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"DISTANCE");
                        FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"POWER_PER_SECOND",mPowerPerSecond);
                        FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"MAXIMAL_POWER",mMaximalPower);
                        this.processTrackWidthTicks();
                        break;
                }


                mOdometer.update();
                mOdometer.log();

                mConfiguration.log();

                mLogger.update();
            }

            mConfiguration.write(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/drive-encoder-tuning.json");
            mLogger.info("Updated configuration saved. You may retrieve it using <b>adb pull /sdcard/FIRST/drive-encoder-tuning.json</b>");
            mLogger.update();
        }
        catch(Exception e) {
            mLogger.error(e.toString());
            mLogger.update();
        }
    }

    void processFwdInPerTick() {

        switch(STEP) {
            case INIT:
                mFwdInPerTick = new InPerTick(mOdometer.forward());
                mFwdInPerTick.init();
                mOdometer.pose(new Pose2d(new Vector2d(0, 0), 0));
                break;

            case PROCESSING:
                mFwdInPerTick.update();
                double ticks = mFwdInPerTick.ticks();
                mLogger.info("Ticks travelled " + ticks);
                mLogger.info("Calculated Forward In Per Ticks" + mInPerTickDistance.get() / ticks);

                break;

            case UPDATE :
                double final_ticks = mFwdInPerTick.ticks();
                mLogger.info("Ticks travelled " + final_ticks);
                mLogger.info("Calculated Forward In Per Ticks" + mInPerTickDistance.get()  / final_ticks);
                mOdometer.forwardInPerTick(mInPerTickDistance.get()  / final_ticks);
                break;
        }

    }

    void processLatInPerTick() {

        switch(STEP) {
            case INIT:
                mLatInPerTick = new InPerTick(mOdometer.lateral());
                mLatInPerTick.init();
                mOdometer.pose(new Pose2d(new Vector2d(0, 0), 0));
                break;

            case PROCESSING:
                mLatInPerTick.update();
                double ticks = mLatInPerTick.ticks();
                mLogger.info("Ticks travelled " + ticks);
                mLogger.info("Calculated Lateral In Per Ticks" + mInPerTickDistance.get() / ticks);
                break;

            case UPDATE :
                double final_ticks = mLatInPerTick.ticks();
                mLogger.info("Ticks travelled " + final_ticks);
                mLogger.info("Calculated Lateral In Per Ticks" + mInPerTickDistance.get()  / final_ticks);
                mOdometer.lateralInPerTick(mInPerTickDistance.get()  / final_ticks);
                break;
        }

    }

    void processTrackWidthTicks() {

        switch(STEP) {
            case STOP:
                FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"START");
                FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"TRACK_WIDTH");
            case INIT:
                mTrackWidthTicks = new TrackWidthTicks(mDriveTrain, mImu, mHardware.voltageSensor(), mLogger);
                mTrackWidthTicks.power(mPowerPerSecond.get(), mMaximalPower.get());
                mOdometer.pose(new Pose2d(new Vector2d(0, 0), 0));
                mFirstTimeCalled = true;
                mShallSave = true;
                FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"START");
                FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"TRACK_WIDTH");
                break;

            case PROCESSING:
                mShallSave = true;
                FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"TRACK_WIDTH");
                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"START",mShallStart);
                if(mFirstTimeCalled) {
                    mTrackWidthTicks.start();
                    mFirstTimeCalled = false;
                }
                if(mShallStart.get()) {
                    mFirstTimeStopped = true;
                    mTrackWidthTicks.update();
                }
                else {
                    if(mFirstTimeStopped) { mTrackWidthTicks.stop(); }
                    mFirstTimeCalled = false;
                }
                mLogger.info("Data collected " + mTrackWidthTicks.data());
                break;

            case UPDATE :
                mFirstTimeCalled = true;
                FtcDashboard.getInstance().removeConfigVariable(this.getClass().getSimpleName(),"START");
                FtcDashboard.getInstance().addConfigVariable(this.getClass().getSimpleName(),"TRACK_WIDTH",mTrackWidth);
                if(mShallSave) {
                    mShallSave = false;
                    mTrackWidthTicks.process();
                }
                mLogger.info("Data collected " + mTrackWidthTicks.data());
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
        public void     set(Double value)    { mDistance = value; }
    }

    // Since double is a simple type, even with the appropriate constructor,
    // mDistance will only be updated locally by the dashboard.
    // We'll have to make sure the code access the mode from the provider using the get
    // Method, since it's the only place the updated information can be found
    static class PowerProvider implements ValueProvider<Double> {
        double mPower;
        @Override
        public Double   get()                { return mPower;  }
        @Override
        public void     set(Double value)    { mPower = value; }
    }

    // ReverseProvider updates the controller reverse configuration
    // Since ConfMotor.Controller is not a simple type, it's managed as
    // pointer, when we change it in the provider, it's changed in the
    // global configuration
    static class StartProvider implements ValueProvider<Boolean> {
        boolean mStart;
        @Override
        public Boolean get()           { return mStart; }
        @Override
        public void set(Boolean value) { mStart = value;   }
    }

    // TrackWidthProvider updates the controller reverse configuration
    // Since ConfMotor.Controller is not a simple type, it's managed as
    // pointer, when we change it in the provider, it's changed in the
    // global configuration
    static class TrackWidthProvider implements ValueProvider<Double> {
        DriveEncodersOdometer   mOdometer;
        Double                  mWidth;
        public TrackWidthProvider(DriveEncodersOdometer odometer) {
            mOdometer = odometer;
            mWidth = 0.0;
        }
        @Override
        public Double get()           { return mWidth; }
        @Override
        public void set(Double value) { mWidth = value; mOdometer.trackWidthTicks(value);   }
    }


}
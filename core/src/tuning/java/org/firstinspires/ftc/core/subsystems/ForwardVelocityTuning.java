/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Forward velocity tuning tool
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.subsystems;

/* System includes */
import java.util.ArrayList;
import java.util.Objects;
import java.util.List;

/* Android includes */
import android.os.Environment;

/* Qualcomm includes */
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/* ACME robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;

/* PedroPathing includes */
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;

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
import org.firstinspires.ftc.core.tuning.Robot;

@Config
@TeleOp(name = "ForwardVelocityTuning", group = "Tuning")
public class ForwardVelocityTuning extends LinearOpMode implements Tuning {

    /* -------- Configuration variables -------- */
    public static String                        CONFIGURATION   = "test";
    public static String                        LEFT_BACK       = "back-left-wheel";
    public static String                        LEFT_FRONT      = "front-left-wheel";
    public static String                        RIGHT_BACK      = "back-right-wheel";
    public static String                        RIGHT_FRONT     = "front-right-wheel";
    public static String                        LOCALIZER       = "mecanum-drive-encoders";
    public static double                        RECORD_NUMBER   = 10;
    public static double                        DISTANCE        = 48;

    /* ---------------- Members ---------------- */
    private LogManager                          mLogger;

    private Configuration                       mConfiguration;
    private String                              mConfigurationName;
    private Hardware                            mHardware;
    private Robot                               mRobot;

    private Controller                          mController;

    private MotorComponent                      mLeftBack;
    private MotorComponent                      mLeftFront;
    private MotorComponent                      mRightBack;
    private MotorComponent                      mRightFront;

    private LocalizerComponent                  mLocalizer;
    private PoseUpdater                         mUpdater;
    private DashboardPoseTracker                mDashboardPoseTracker;
    
    private List<Double>                        mVelocities;
    private boolean                             mShallEnd;

    private SaveProvider                        mShallSave;


    @Override
    public void runOpMode() {

        try {

            mLogger = new LogManager(null,FtcDashboard.getInstance(),"forward-velocity-tuning");
            mLogger.level(LogManager.Severity.TRACE);

            mController = new Controller(gamepad1, mLogger);

            mRobot    = new Robot(this, hardwareMap, mLogger);

            mConfigurationName = CONFIGURATION;
            mConfiguration = new Configuration(mLogger);
            mConfiguration.register("robot", mRobot);
            mConfiguration.read(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/" + mConfigurationName + ".json");
            mConfiguration.log();

            mHardware   = mRobot.hardware(this);
            mLeftBack   = null;
            mRightBack  = null;
            mLeftFront  = null;
            mRightFront = null;
            if(mHardware.motors().containsKey(LEFT_BACK))   { mLeftBack = mHardware.motors().get(LEFT_BACK);        }
            if(mHardware.motors().containsKey(RIGHT_BACK))  { mRightBack = mHardware.motors().get(RIGHT_BACK);      }
            if(mHardware.motors().containsKey(LEFT_FRONT))  { mLeftFront = mHardware.motors().get(LEFT_FRONT);      }
            if(mHardware.motors().containsKey(RIGHT_FRONT)) { mRightFront = mHardware.motors().get(RIGHT_FRONT);    }
            if(mLeftBack == null) { mLogger.error("Missing left back motor with name " + LEFT_BACK); }
            if(mRightBack == null) { mLogger.error("Missing right back motor with name " + RIGHT_BACK); }
            if(mLeftFront == null) { mLogger.error("Missing left front motor with name " + LEFT_FRONT); }
            if(mRightFront == null) { mLogger.error("Missing right front motor with name " + RIGHT_FRONT); }

            mLocalizer = null;
            if(mHardware.localizers().containsKey(LOCALIZER))   { mLocalizer = mHardware.localizers().get(LOCALIZER);        }
            if(mLocalizer == null) { mLogger.error("Missing localizer with name " + LOCALIZER); }
            else {
                mUpdater = new PoseUpdater(null, mLocalizer);
                mDashboardPoseTracker = new DashboardPoseTracker(mUpdater);

                mLocalizer.setStartPose(new Pose(0,0,0));
                mLocalizer.setPose(new Pose(0,0,0));
            }

            mVelocities = new ArrayList<>();
            for (int i = 0; i < RECORD_NUMBER; i++) {
                mVelocities.add(0.0);
            }
            mShallEnd   = false;

            mShallSave = new SaveProvider();
            FtcDashboard.getInstance().removeConfigVariable(ForwardVelocityTuning.class.getSimpleName(),"SAVE");

            String description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                    "<p style=\"font-weight: bold; font-size: 14px\">The robot will run at 1 power until it reaches " + DISTANCE + " inches forward</p>" +
                    "<p style=\"font-weight: bold; font-size: 14px\">Make sure you have enough room, since the robot has inertia after cutting power</p>" +
                    "<p style=\"font-weight: bold; font-size: 14px\">After running the distance, the robot will cut power from the drivetrain and display the forward velocity</p>" +
                    "<p style=\"font-weight: bold; font-size: 14px\">Press A on game pad 1 to stop..</p>";
            mLogger.info(LogManager.Target.DASHBOARD,description);
            mLogger.metric("pose", ""+mUpdater.getPose());

            FtcDashboard.getInstance().updateConfig();
            mLogger.update();

            waitForStart();

            mLogger.clear();

            startMotors();

            while(opModeIsActive()) {

                double average = 0;
                for (Double velocity : mVelocities) {
                    average += velocity;
                }
                if(!mVelocities.isEmpty()) { average /=  mVelocities.size(); }
                if(!mVelocities.isEmpty()) { mLogger.metric("Average velocity","" + average); }
                mLogger.metric("shall end", ""+mShallEnd);
                mLogger.metric("Collected","" + mVelocities.size());

                /* Manage motor change */
                if(!Objects.equals(mLeftBack.name(), LEFT_BACK)) {
                    stopMotors();
                    if(mHardware.motors().containsKey(LEFT_BACK))   { mLeftBack = mHardware.motors().get(LEFT_BACK);        }
                    if(mLeftBack == null) { mLogger.error("Missing left back motor with name " + LEFT_BACK); }
                    mVelocities.clear();
                    mUpdater.setPose(new Pose(0,0,0));
                    startMotors();
                }
                if(!Objects.equals(mLeftFront.name(), LEFT_FRONT)) {
                    stopMotors();
                    if(mHardware.motors().containsKey(LEFT_FRONT))   { mLeftFront = mHardware.motors().get(LEFT_FRONT);        }
                    if(mLeftFront == null) { mLogger.error("Missing left front motor with name " + LEFT_FRONT); }
                    mVelocities.clear();
                    mUpdater.setPose(new Pose(0,0,0));
                    startMotors();
                }
                if(!Objects.equals(mRightBack.name(), RIGHT_BACK)) {
                    stopMotors();
                    if(mHardware.motors().containsKey(RIGHT_BACK))   { mRightBack = mHardware.motors().get(RIGHT_BACK);        }
                    if(mRightBack == null) { mLogger.error("Missing right back motor with name " + RIGHT_BACK); }
                    mVelocities.clear();
                    mUpdater.setPose(new Pose(0,0,0));
                    startMotors();
                }
                if(!Objects.equals(mRightFront.name(), RIGHT_FRONT)) {
                    stopMotors();
                    if(mHardware.motors().containsKey(RIGHT_FRONT))   { mRightFront = mHardware.motors().get(RIGHT_FRONT);        }
                    if(mRightFront == null) { mLogger.error("Missing right front motor with name " + RIGHT_FRONT); }
                    mVelocities.clear();
                    mUpdater.setPose(new Pose(0,0,0));
                    startMotors();
                }
                if(!Objects.equals(mLocalizer.name(), LOCALIZER)) {
                    stopMotors();
                    if(mHardware.localizers().containsKey(LOCALIZER)) { mLocalizer = mHardware.localizers().get(LOCALIZER);        }
                    if(mLocalizer == null) { mLogger.error("Missing localizer with name " + LOCALIZER); }
                    mVelocities.clear();
                    mUpdater.setPose(new Pose(0,0,0));
                    startMotors();
                }

                if (mController.buttons.a.pressedOnce()) {
                    mShallEnd = true;
                }

                if(!mShallEnd) {
                    if (Math.abs(mUpdater.getPose().getX()) > DISTANCE) {
                        mShallEnd = true;
                    } else {
                        double currentVelocity = Math.abs(MathFunctions.dotProduct(mUpdater.getVelocity(), new Vector(1, 0)));
                        mVelocities.add(currentVelocity);
                        mVelocities.remove(0);
                    }
                }

                if(mShallEnd) {

                    stopMotors();
                    FtcDashboard.getInstance().addConfigVariable(ForwardVelocityTuning.class.getSimpleName(),"SAVE",mShallSave);

                    if(mShallSave.get()) {
                        FollowerConstants.xMovement = average;
                    }
                }

                mLocalizer.update();
                mUpdater.update();
                mDashboardPoseTracker.update();

                // Log motors state and updated configuration
                this.logLocalizerState(mLogger);
                mConfiguration.log();

                Drawing.drawPoseHistory(mDashboardPoseTracker, "#4CAF50");
                Drawing.drawRobot(mUpdater.getPose(), "#4CAF50");
                Drawing.sendPacket();

                mLogger.update();
            }

            mConfiguration.write(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/forward-velocity-tuning.json");
            mLogger.info("Updated configuration saved. You may retrieve it using <b>adb pull /sdcard/FIRST/forward-velocity-tuning.json</b>");
            mLogger.update();

            mLogger.stop();
        }
        catch(Exception e) {
            mLogger.error(e.toString());
            mLogger.update();
        }
    }

    private void stopMotors() {
        if(mRightFront != null) { mRightFront.power(0); }
        if(mRightFront != null) { mRightFront.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
        if(mLeftFront != null) { mLeftFront.power(0); }
        if(mLeftFront != null) { mLeftFront.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
        if(mRightBack != null) { mRightBack.power(0); }
        if(mRightBack != null) { mRightBack.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
        if(mLeftBack != null) { mLeftBack.power(0); }
        if(mLeftBack != null) { mLeftBack.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
    }

    private void startMotors() {

        if(mRightFront != null && mRightBack != null && mLeftFront != null && mLeftBack != null) {
            mRightFront.power(1.0);
            mRightFront.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            mRightFront.achieveableMaxRPMFraction(1.0);
            mLeftFront.power(1.0);
            mLeftFront.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            mLeftFront.achieveableMaxRPMFraction(1.0);
            mRightBack.power(1.0);
            mRightBack.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            mRightBack.achieveableMaxRPMFraction(1.0);
            mLeftBack.power(1.0);
            mLeftBack.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            mLeftBack.achieveableMaxRPMFraction(1.0);
        }
    }

    private void logLocalizerState(LogManager logger) {

        if (mUpdater != null) {

            double heading = mUpdater.getPose().getHeading();
            if(heading > Math.PI) { heading -= 2 * Math.PI; }

            logger.info("-----> x : " +  mUpdater.getPose().getX());
            logger.info("-----> y : " +  mUpdater.getPose().getY());
            logger.info("-----> heading : " +  heading);
            logger.info("-----> total heading : " +  mUpdater.getTotalHeading());

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
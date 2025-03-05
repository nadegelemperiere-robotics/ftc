/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Follower final test following a triangle
   ------------------------------------------------------- */
package org.firstinspires.ftc.core;

/* Android includes */
import android.os.Environment;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/* ACME includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/* PedroPathing includes */
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.localization.Pose;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Subsystems includes */
import org.firstinspires.ftc.core.subsystems.MecanumDrive;

/* Tuning includes */
import org.firstinspires.ftc.core.tuning.Tuning;
import org.firstinspires.ftc.core.tuning.Robot;

/**
 * This is the CurvedBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * forward and to the left. On reaching the end of the forward Path, the robot runs the backward
 * Path the same distance back to the start. Rinse and repeat! This is good for testing a variety
 * of Vectors, like the drive Vector, the translational Vector, the heading Vector, and the
 * centripetal Vector. Remember to test your tunings on StraightBackAndForth as well, since tunings
 * that work well for curves might have issues going in straight lines.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@Config
@Autonomous (name = "TriangleTest", group = "Test")
public class TriangleTest extends LinearOpMode implements Tuning {

    /* -------- Configuration variables -------- */
    public static double    MAX_POWER           = 1;
    public static double    LENGTH              = 24;
    public static String    DRIVE_TRAIN         = "drive-train";

    /* ---------------- Members ---------------- */
    private LogManager      mLogger;

    private Configuration   mConfiguration;
    private Robot           mRobot;

    private MecanumDrive    mDrive;
    private double          mMaxPower;

    private PathChain       mTriangle;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void runOpMode() {

        try {

            mLogger = new LogManager(null,FtcDashboard.getInstance(),"mecanum-drive-straight-back-and-forth-tuning");
            mLogger.level(LogManager.Severity.TRACE);

            mRobot = new Robot(this, hardwareMap, mLogger);

            mConfiguration = new Configuration(mLogger);
            mConfiguration.register("robot", mRobot);
            mConfiguration.read();
            mConfiguration.log();

            mDrive = (MecanumDrive)mRobot.subsystem(this, DRIVE_TRAIN);
            if(mDrive != null) {
                mDrive.setStartingPose(new Pose(0,0,0));
                mDrive.setPose(new Pose(0,0,0));
                mMaxPower = MAX_POWER;
                mDrive.setMaxPower(mMaxPower);
                FollowerConstants.maxPower = mMaxPower;
            }

            Pose startPose = new Pose(0,0, Math.toRadians(0));
            Pose interPose = new Pose(LENGTH, -LENGTH, Math.toRadians(90));
            Pose endPose = new Pose(LENGTH, LENGTH, Math.toRadians(45));

            if(mDrive != null) {
                mTriangle = mDrive.pathBuilder()
                        .addPath(new BezierLine(new Point(startPose), new Point(interPose)))
                        .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
                        .addPath(new BezierLine(new Point(interPose), new Point(endPose)))
                        .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
                        .addPath(new BezierLine(new Point(endPose), new Point(startPose)))
                        .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                        .build();

                mDrive.followPath(mTriangle);
            }

            String description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                    "<p style=\"font-weight: bold; font-size: 14px\"> This will run in a roughly triangular shape," +
                    " starting on the bottom-middle point. So, make sure you have enough " +
                    " space to the left, front, and right to run the OpMode. </p>";
            mLogger.info(LogManager.Target.DASHBOARD,description);

            FtcDashboard.getInstance().updateConfig();

            mLogger.update();

            waitForStart();

            mLogger.clear();

            while(opModeIsActive()) {

                if(mDrive != null) {
                    mDrive.update();
                    if (mDrive.atParametricEnd()) {
                        mDrive.followPath(mTriangle);
                    }

                    if (MAX_POWER != mMaxPower) {
                        mMaxPower = MAX_POWER;
                        mDrive.setMaxPower(mMaxPower);
                        FollowerConstants.maxPower = mMaxPower;
                        FtcDashboard.getInstance().updateConfig();
                    }
                }

                mRobot.log();

                mLogger.update();
            }

            mConfiguration.log();
            mConfiguration.write(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/mecanum-drive-straight-back-and-forth-tuning.json");
            mLogger.info("Updated configuration saved. You may retrieve it using <b>adb pull /sdcard/FIRST/mecanum-drive-straight-back-and-forth-tuning.json</b>");
            mLogger.update();
            mLogger.stop();
        }
        catch(Exception e) {
            mLogger.error(e.toString());
            mLogger.update();
        }
    }
}

package org.firstinspires.ftc.core.subsystems;

/* Android includes */
import android.os.Environment;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/* ACME includes */
import com.pedropathing.util.Drawing;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/* PedroPathing includes */
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.localization.Pose;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

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
@Autonomous (name = "StraightBackAndForthTuning", group = "Tuning")
public class StraightBackAndForthTuning extends LinearOpMode implements Tuning {

    /* -------- Configuration variables -------- */
    public static String    CONFIGURATION       = "test";
    public static double    DISTANCE            = 20;
    public static String    DRIVE_TRAIN         = "drive-train";
    public static boolean   USE_TRANSLATIONAL   = true;
    public static boolean   USE_CENTRIPETAL     = true;
    public static boolean   USE_HEADING         = true;
    public static boolean   USE_DRIVE           = true;

    /* ---------------- Members ---------------- */
    private LogManager      mLogger;

    private Configuration   mConfiguration;
    private String          mConfigurationName;
    private Robot           mRobot;

    private MecanumDrive    mDrive;

    private boolean         mForward = true;

    private Path            mForwards;
    private Path            mBackwards;

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

            mConfigurationName = CONFIGURATION;
            mConfiguration = new Configuration(mLogger);
            mConfiguration.register("robot", mRobot);
            mConfiguration.read(Environment.getExternalStorageDirectory().getPath()
                    + "/FIRST/" + mConfigurationName + ".json");
            mConfiguration.log();

            mDrive = (MecanumDrive)mRobot.subsystem(this, DRIVE_TRAIN);
            if(mDrive != null) {
                mDrive.useDrive(USE_DRIVE);
                mDrive.useHeading(USE_HEADING);
                mDrive.useTranslational(USE_TRANSLATIONAL);
                mDrive.useCentripetal(USE_CENTRIPETAL);
                mDrive.drawOnDashboard(true);
                mDrive.setStartingPose(new Pose(0,0,0));
                mDrive.setPose(new Pose(0,0,0));
            }

            mForwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
            mForwards.setConstantHeadingInterpolation(0);
            mBackwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
            mBackwards.setConstantHeadingInterpolation(0);

            mDrive.followPath(mForwards);

            String description = "<p style=\"font-weight: bold; font-size: 14px\"> ------------------------- </p>" +
                    "<p style=\"font-weight: bold; font-size: 14px\"> This will run the robot in a straight line going " + DISTANCE + " inches" +
                    " forward. The robot will go forward and backward continuously along " +
                    " the path. Make sure you have enough room. </p>";
            mLogger.info(LogManager.Target.DASHBOARD,description);

            FtcDashboard.getInstance().updateConfig();
            mLogger.update();

            waitForStart();

            mLogger.clear();

            while(opModeIsActive()) {

                mDrive.update();
                if (!mDrive.isBusy()) {
                    if (mForward) {
                        mForward = false;
                        mDrive.followPath(mBackwards);
                    } else {
                        mForward = true;
                        mDrive.followPath(mForwards);
                    }
                }

                mDrive.telemetryDebug();

                mLogger.metric("Going forward","" + mForward);

                Drawing.drawPoseHistory(mDrive.getDashboardPoseTracker(), "#4CAF50");
                Drawing.drawRobot(mDrive.getPose(), "#4CAF50");
                Drawing.sendPacket();

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

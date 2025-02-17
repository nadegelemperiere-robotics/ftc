/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localization using 4 wheels encoders
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.odometers;

/* System includes */
import java.util.LinkedList;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/* ACME robotics includes */
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.imus.ImuComponent;
import org.firstinspires.ftc.core.components.motors.MotorComponent;

public class DriveEncodersOdometer implements OdometerComponent {

    static  final String    sLeftFrontHwMapKey   = "front-left-wheel";
    static  final String    sLeftBackHwMapKey    = "back-left-wheel";
    static  final String    sRightFrontHwMapKey  = "front-right-wheel";
    static  final String    sRightBackHwMapKey   = "back-right-wheel";
    static  final String    sImuHwMapKey         = "imu";
    static  final String    sTrackWidthTicksKey  = "track-width-ticks";
    static  final String    sLateralInPerTickKey = "lateral-in-per-tick";
    static  final String    sInPerTickKey        = "in-per-tick";

    LogManager                  mLogger;

    boolean                     mConfigurationValid;
    boolean                     mIsFirstTime;

    String                      mName;
    String                      mLeftBackHwName;
    String                      mLeftFrontHwName;
    String                      mRightBackHwName;
    String                      mRightFrontHwName;
    String                      mImuHwName;

    HardwareMap                 mMap;
    Encoder                     mLeftFront;
    Encoder                     mRightFront;
    Encoder                     mLeftBack;
    Encoder                     mRightBack;
    ImuComponent                mImu;

    Pose2d                      mInitialPose;
    Pose2d                      mCurrentPose;
    PoseVelocity2d              mCurrentVelocity;
    public LinkedList<Pose2d>   mPoseHistory;

    double                      mInPerTick;
    double                      mTrackWidthTicks;
    double                      mLateralInPerTick;

    double                      mLastHeading;
    double                      mLastLeftFrontPos;
    double                      mLastLeftBackPos;
    double                      mLastRightFrontPos;
    double                      mLastRightBackPos;

    MecanumKinematics           mKinematics;

    public  DriveEncodersOdometer(String name, HardwareMap hwMap, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = true;
        mIsFirstTime        = true;

        mName               = name;
        mLeftFrontHwName    = "";
        mLeftBackHwName     = "";
        mRightFrontHwName   = "";
        mRightBackHwName    = "";
        mImuHwName          = "";

        mCurrentPose     = new Pose2d(new Vector2d(0,0),0);
        mInitialPose     = new Pose2d(new Vector2d(0,0),0);
        mCurrentVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
        mPoseHistory     = new LinkedList<>();

        mMap        = hwMap;
        mLeftFront  = null;
        mLeftBack   = null;
        mRightFront = null;
        mRightBack  = null;
        mImu        = null;


    }

    @Override
    public void         pose(Pose2d current) {
        if(mConfigurationValid) {
            mIsFirstTime = true;
            mInitialPose = current;
        }
    }

    @Override
    public void         update() {
        if (mConfigurationValid) {

            Twist2dDual<Time> twist;

            PositionVelocityPair leftFrontPosVel = mLeftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = mLeftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = mRightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = mRightFront.getPositionAndVelocity();

            double heading = mImu.heading();

            if (!mIsFirstTime) {
                mIsFirstTime = true;


                twist = new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }
            else {

                double headingDelta = heading - mLastHeading;
                twist = mKinematics.forward(new MecanumKinematics.WheelIncrements<>(
                        new DualNum<Time>(new double[]{
                                (leftFrontPosVel.position - mLastLeftFrontPos),
                                leftFrontPosVel.velocity,
                        }).times(mInPerTick),
                        new DualNum<Time>(new double[]{
                                (leftBackPosVel.position - mLastLeftBackPos),
                                leftBackPosVel.velocity,
                        }).times(mInPerTick),
                        new DualNum<Time>(new double[]{
                                (rightBackPosVel.position - mLastRightBackPos),
                                rightBackPosVel.velocity,
                        }).times(mInPerTick),
                        new DualNum<Time>(new double[]{
                                (rightFrontPosVel.position - mLastRightFrontPos),
                                rightFrontPosVel.velocity,
                        }).times(mInPerTick)
                ));

                twist = new Twist2dDual<>(
                        twist.line,
                        DualNum.cons(headingDelta, twist.angle.drop(1))
                );
            }


            mLastLeftFrontPos = leftFrontPosVel.position;
            mLastLeftBackPos = leftBackPosVel.position;
            mLastRightBackPos = rightBackPosVel.position;
            mLastRightFrontPos = rightFrontPosVel.position;

            mLastHeading = heading;

            mCurrentPose = mCurrentPose.plus(twist.value());
            mCurrentVelocity = twist.velocity().value();

            mPoseHistory.add(mCurrentPose);
            while (mPoseHistory.size() > 100) {
                mPoseHistory.removeFirst();
            }

        }
    }

    @Override
    public Pose2d           pose() {
        return mCurrentPose;
    }

    @Override
    public PoseVelocity2d   velocity() { return mCurrentVelocity; }

    @Override
    public void             log() {
        if (mConfigurationValid) {
            mLogger.metric("x", mCurrentPose.position.x + " inches");
            mLogger.metric("y", mCurrentPose.position.y + " inches");
            mLogger.metric("heading", mCurrentPose.heading.toDouble() + " rad");

            mLogger.metric("vx",mCurrentVelocity.linearVel.x + " inches/s");
            mLogger.metric("vy",mCurrentVelocity.linearVel.y + " inches/s");
            mLogger.metric("vheading",mCurrentVelocity.angVel + " rad/s");
        }
    }

    /* ------------------ Configurable functions ------------------- */

    /**
     * Determines if the driver odometer component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /**
     * Reads and applies the drive odometer configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    @Override
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;

        mLeftFront  = null;
        mLeftBack   = null;
        mRightFront = null;
        mRightBack  = null;
        mImu        = null;

        try {

            if(mMap != null && reader.has(sLeftBackHwMapKey)) {
                mLeftBackHwName = reader.getString(sLeftBackHwMapKey);
//                DcMotorEx motor = mMap.tryGet(DcMotorEx.class,conf.get(0).mapName());
//                if(motor != null) { mLeftFront = new OverflowEncoder(new RawEncoder(motor)); }
//                if(mLeftFront != null && MotorComponent.sString2Direction.containsKey(conf.get(0).direction())) {
//                    mLeftFront.setDirection(Objects.requireNonNull(MotorComponent.sString2Direction.get(conf.get(0).direction())));
//                }
//                else if(mLeftFront != null)                {
//                    mLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
//                }
                // Should be better to get it from motor list
            }
            if(reader.has(sLeftFrontHwMapKey)) {
                mLeftFrontHwName = reader.getString(sLeftFrontHwMapKey);
            }
            if(reader.has(sRightBackHwMapKey)) {
                mRightBackHwName = reader.getString(sRightBackHwMapKey);
            }
            if(reader.has(sRightFrontHwMapKey)) {
                mRightFrontHwName = reader.getString(sRightFrontHwMapKey);
            }
            if(reader.has(sImuHwMapKey)) {
                mImuHwName = reader.getString(sImuHwMapKey);
            }

            mInPerTick  = 1.0;
            mLateralInPerTick  = 1.0;
            mTrackWidthTicks = 1.0;

            if (reader.has(sInPerTickKey)) {
                mInPerTick = reader.getDouble(sInPerTickKey);
            }
            if (reader.has(sTrackWidthTicksKey)) {
                mTrackWidthTicks = reader.getDouble(sTrackWidthTicksKey);
            }
            if (reader.has(sLateralInPerTickKey)) {
                mLateralInPerTick = reader.getDouble(sLateralInPerTickKey);
            }

            mKinematics = new MecanumKinematics(
                    mInPerTick * mTrackWidthTicks, mInPerTick / mLateralInPerTick);

            Pose2d origin = new Pose2d(new Vector2d(0,0),0);

            this.pose(origin);

        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

        if(mLeftFront == null)  { mConfigurationValid = false; }
        if(mLeftBack == null)   { mConfigurationValid = false; }
        if(mRightFront == null) { mConfigurationValid = false; }
        if(mRightBack == null)  { mConfigurationValid = false; }
        if(mImu == null)  { mConfigurationValid = false; }

    }

    /**
     * Writes the current drive odometer configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) {

        if(mConfigurationValid) {

            try {

                writer.put(sLeftBackHwMapKey,mLeftBackHwName);
                writer.put(sLeftFrontHwMapKey,mLeftFrontHwName);
                writer.put(sRightBackHwMapKey,mRightBackHwName);
                writer.put(sRightFrontHwMapKey,mRightFrontHwName);
                writer.put(sImuHwMapKey,mImuHwName);
                writer.put(sInPerTickKey,mInPerTick);
                writer.put(sTrackWidthTicksKey,mTrackWidthTicks);
                writer.put(sLateralInPerTickKey,mLateralInPerTick);

            } catch (JSONException e) { mLogger.error(e.getMessage()); }
        }

    }

    /**
     * Generates an HTML representation of the drive odometer configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted drive odometer configuration.
     */
    @Override
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {

            result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                    .append(" - HW : LF ")
                    .append(mLeftFrontHwName)
                    .append(" LB ")
                    .append(mLeftBackHwName)
                    .append(" RF ")
                    .append(mRightFrontHwName)
                    .append(" RB ")
                    .append(mRightBackHwName)
                    .append(" IMU ")
                    .append(mImuHwName)
                    .append(" - IPT : ")
                    .append(mInPerTick)
                    .append(" - LIPT : ")
                    .append(mLateralInPerTick)
                    .append(" - TWT : ")
                    .append(mTrackWidthTicks)
                    .append("</li>\n");

        }

        return result.toString();

    }

    /**
     * Generates a text-based representation of the drive odometer configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted drive odometer configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {

            result.append(header)
                    .append("> HW : LF ")
                    .append(mLeftFrontHwName)
                    .append(" LB ")
                    .append(mLeftBackHwName)
                    .append(" RF ")
                    .append(mRightFrontHwName)
                    .append(" RB ")
                    .append(mRightBackHwName)
                    .append(" IMU ")
                    .append(mImuHwName)
                    .append(" - IPT : ")
                    .append(mInPerTick)
                    .append(" - LIPT : ")
                    .append(mLateralInPerTick)
                    .append(" - TWT : ")
                    .append(mTrackWidthTicks)
                    .append("\n");
        }

        return result.toString();

    }

}

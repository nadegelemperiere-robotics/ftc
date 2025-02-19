/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localization using 2 odometry wheels and and IMU
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.odometers;

/* System includes */
import java.util.LinkedList;
import java.util.Map;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* ACME robotics includes */
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.Encoder;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.imus.ImuComponent;
import org.firstinspires.ftc.core.components.motors.MotorComponent;

public class TwoDeadWheelsOdometer implements OdometerComponent {

    static  final String sParHwMapKey   = "par";
    static  final String sPerpHwMapKey  = "perp";
    static  final String sImuHwMapKey   = "imu";
    static  final String sParTicksKey   = "par-y-ticks";
    static  final String sPerpTicksKey  = "perp-x-ticks";
    static  final String sInPerTickKey  = "in-per-tick";

    final LogManager            mLogger;

    boolean                     mConfigurationValid;
    boolean                     mIsFirstTime;

    final String                mName;
    String                      mParHwMapName;
    String                      mPerpHwMapName;
    String                      mImuHwMapName;

    final HardwareMap           mMap;
    Encoder                     mPar;
    Encoder                     mPerp;
    ImuComponent                mImu;

    Pose2d                      mInitialPose;
    Pose2d                      mCurrentPose;
    PoseVelocity2d              mCurrentVelocity;
    final LinkedList<Pose2d>    mPoseHistory;

    double                      mParYTicks;
    double                      mPerpXTicks;
    double                      mInPerTick;

    double                      mLastHeading;
    double                      mLastHeadingVel;
    double                      mVelocityOffset;
    double                      mLastParPos;
    double                      mLastPerpPos;


    public  TwoDeadWheelsOdometer(String name, HardwareMap hwMap, Map<String, MotorComponent> motors, Map<String,ImuComponent> imus, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = true;
        mIsFirstTime        = true;

        mName           = name;
        mParHwMapName   = "";
        mPerpHwMapName  = "";
        mImuHwMapName   = "";

        mCurrentPose     = new Pose2d(new Vector2d(0,0),0);
        mCurrentVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
        mPoseHistory     = new LinkedList<>();

        mMap    = hwMap;
        mPar    = null;
        mPerp   = null;
        mImu    = null;

    }

    @Override
    public void     pose(Pose2d current) {
        if(mConfigurationValid) {
            mInitialPose = current;
        }
    }

    @Override
    public void     update() {

        if (mConfigurationValid) {

            Twist2dDual<Time> twist;

            PositionVelocityPair parPosVel = mPar.getPositionAndVelocity();
            PositionVelocityPair perpPosVel = mPerp.getPositionAndVelocity();

            double heading = mImu.heading();
            double headingVelocity = mImu.headingVelocity();
            if (Math.abs(headingVelocity - mLastHeadingVel) > Math.PI) {
                mVelocityOffset -= Math.signum(headingVelocity) * 2 * Math.PI;
            }
            mLastHeadingVel = headingVelocity;
            headingVelocity += mVelocityOffset;

            if (mIsFirstTime) {
                mIsFirstTime = false;

                twist = new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }
            else {

                double parPosDelta  = parPosVel.position - mLastParPos;
                double perpPosDelta = perpPosVel.position - mLastPerpPos;
                double headingDelta = heading - mLastHeading;

                twist = new Twist2dDual<>(
                        new Vector2dDual<>(
                                new DualNum<Time>(new double[]{
                                        parPosDelta - mParYTicks * headingDelta,
                                        parPosVel.velocity - mParYTicks * headingVelocity,
                                }).times(mInPerTick),
                                new DualNum<Time>(new double[]{
                                        perpPosDelta - mPerpXTicks * headingDelta,
                                        perpPosVel.velocity - mPerpXTicks * headingVelocity,
                                }).times(mInPerTick)
                        ),
                        new DualNum<>(new double[]{
                                headingDelta,
                                headingVelocity,
                        })
                );
            }

            mLastParPos = parPosVel.position;
            mLastPerpPos = perpPosVel.position;
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
    public Pose2d           pose() { return mCurrentPose; }

    @Override
    public PoseVelocity2d   velocity() { return mCurrentVelocity;    }


    @Override
    public void             log() {
        if (mConfigurationValid) {
            mLogger.metric(mName + "-x", mCurrentPose.position.x + " inches");
            mLogger.metric(mName + "-y", mCurrentPose.position.y + " inches");
            mLogger.metric(mName + "-heading", mCurrentPose.heading.toDouble() + " rad");

            mLogger.metric(mName + "-vx",mCurrentVelocity.linearVel.x + " inches/s");
            mLogger.metric(mName + "-vy",mCurrentVelocity.linearVel.y + " inches/s");
            mLogger.metric(mName + "-vheading",mCurrentVelocity.angVel + " rad/s");
        }
    }


    /* ------------------ Configurable functions ------------------- */

    /**
     * Determines if the 2 dead wheel component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /**
     * Reads and applies the 2 dead wheel configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    @Override
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        mPar = null;
        mImu = null;
        mPerp = null;

        try {

            if(mMap != null && reader.has(sParHwMapKey)) {
                mParHwMapName = reader.getString(sParHwMapKey);
//                DcMotorEx motor = hwMap.tryGet(DcMotorEx.class,mParHwMapName);
//                if(motor != null) { mPar = new OverflowEncoder(new RawEncoder(motor)); }
//                if(mPar != null && MotorComponent.sString2Direction.containsKey(conf.get(0).direction())) {
//                    mPar.setDirection(Objects.requireNonNull(MotorComponent.sString2Direction.get(conf.get(0).direction())));
//                }
//                else if(mPar != null)                {
//                    mPar.setDirection(DcMotorSimple.Direction.FORWARD);
//                }
            }
            if(mMap != null && reader.has(sPerpHwMapKey)) {
                mPerpHwMapName = reader.getString(sPerpHwMapKey);
            }
            if(mMap != null && reader.has(sImuHwMapKey)) {
                mImuHwMapName = reader.getString(sImuHwMapKey);
            }

            mParYTicks = 0.0;
            mPerpXTicks = 0.0;
            mInPerTick  = 1.0;

            if (reader.has(sParTicksKey)) {
                mParYTicks = reader.getDouble(sParTicksKey);
            }
            if (reader.has(sPerpTicksKey)) {
                mPerpXTicks = reader.getDouble(sPerpTicksKey);
            }
            if (reader.has(sInPerTickKey)) {
                mInPerTick = reader.getDouble(sInPerTickKey);
            }

            Pose2d origin = new Pose2d(new Vector2d(0,0),0);

            this.pose(origin);
        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

        if(mPar == null) { mConfigurationValid = false; }
        if(mPerp == null) { mConfigurationValid = false; }
        if(mImu == null) { mConfigurationValid = false; }

    }

    /**
     * Writes the current 3 dead wheel odometer configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) {

        if(mConfigurationValid) {

            try {

                writer.put(sParHwMapKey,mParHwMapName);
                writer.put(sPerpHwMapKey,mPerpHwMapName);
                writer.put(sImuHwMapKey,mImuHwMapName);
                writer.put(sInPerTickKey,mInPerTick);
                writer.put(sParTicksKey,mParYTicks);
                writer.put(sPerpTicksKey,mPerpXTicks);

            } catch (JSONException e) { mLogger.error(e.getMessage()); }
        }

    }

    /**
     * Generates an HTML representation of the 3 dead wheel configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted 3 dead wheel configuration.
     */
    @Override
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {

            result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                    .append(" - HW : Par ")
                    .append(mParHwMapName)
                    .append(" Perp ")
                    .append(mPerpHwMapName)
                    .append(" Imu ")
                    .append(mImuHwMapName)
                    .append(" - IPT : ")
                    .append(mInPerTick)
                    .append(" - PAT : ")
                    .append(mParYTicks)
                    .append(" - PPT : ")
                    .append(mPerpXTicks)
                    .append("</li>\n");


        }

        return result.toString();

    }

    /**
     * Generates a text-based representation of the 3 dead wheel odometer configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted 3 dead wheel odometer configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {

            result.append(header)
                    .append("> HW : Par ")
                    .append(mParHwMapName)
                    .append(" Perp ")
                    .append(mPerpHwMapName)
                    .append(" Imu ")
                    .append(mImuHwMapName)
                    .append(" - IPT : ")
                    .append(mInPerTick)
                    .append(" - PAT : ")
                    .append(mParYTicks)
                    .append(" - PPT : ")
                    .append(mPerpXTicks)
                    .append("\n");
        }

        return result.toString();

    }

}

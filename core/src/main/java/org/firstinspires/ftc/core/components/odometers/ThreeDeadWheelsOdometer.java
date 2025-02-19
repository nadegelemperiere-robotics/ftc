/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localization using 3 odometry wheels
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.odometers;

/* System includes */
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/* ACME robotics includes */
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.Encoder;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.motors.MotorComponent;

public class ThreeDeadWheelsOdometer implements OdometerComponent {

    static  final String    sPar0HwMapKey  = "par0";
    static  final String    sPar1HwMapKey  = "par1";
    static  final String    sPerpHwMapKey  = "perp";
    static  final String    sPar0TicksKey  = "par0-y-ticks";
    static  final String    sPar1TicksKey  = "par1-y-ticks";
    static  final String    sPerpTicksKey  = "perp-x-ticks";
    static  final String    sInPerTickKey  = "in-per-tick";

    final LogManager            mLogger;

    boolean                     mConfigurationValid;
    boolean                     mIsFirstTime;

    final String                mName;
    String                      mPar0HwMapName;
    String                      mPar1HwMapName;
    String                      mPerpHwMapName;

    final HardwareMap           mMap;
    Encoder                     mPar0;
    Encoder                     mPar1;
    Encoder                     mPerp;

    Pose2d                      mInitialPose;
    Pose2d                      mCurrentPose;
    PoseVelocity2d              mCurrentVelocity;
    final LinkedList<Pose2d>    mPoseHistory;

    double                      mPar0YTicks;
    double                      mPar1YTicks;
    double                      mPerpXTicks;
    double                      mInPerTick;

    double                      mLastPar0Pos;
    double                      mLastPar1Pos;
    double                      mLastPerpPos;

    public  ThreeDeadWheelsOdometer(String name, HardwareMap hwMap, Map<String, MotorComponent> motors, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;

        mName           = name;
        mPar0HwMapName  = "";
        mPar1HwMapName  = "";
        mPerpHwMapName  = "";

        mCurrentPose     = new Pose2d(new Vector2d(0,0),0);
        mCurrentVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
        mPoseHistory     = new LinkedList<>();

        mMap  = hwMap;
        mPar0 = null;
        mPar1 = null;
        mPerp = null;
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

            PositionVelocityPair par0PosVel = mPar0.getPositionAndVelocity();
            PositionVelocityPair par1PosVel = mPar1.getPositionAndVelocity();
            PositionVelocityPair perpPosVel = mPerp.getPositionAndVelocity();

            if (mIsFirstTime) {
                mIsFirstTime = false;

                twist =  new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }
            else {

                double par0PosDelta = par0PosVel.position - mLastPar0Pos;
                double par1PosDelta = par1PosVel.position - mLastPar1Pos;
                double perpPosDelta = perpPosVel.position - mLastPerpPos;

                twist = new Twist2dDual<>(
                        new Vector2dDual<>(
                                new DualNum<Time>(new double[]{
                                        (mPar0YTicks * par1PosDelta - mPar1YTicks * par0PosDelta) / (mPar0YTicks - mPar1YTicks),
                                        (mPar0YTicks * par1PosVel.velocity - mPar1YTicks * par0PosVel.velocity) / (mPar0YTicks - mPar1YTicks),
                                }).times(mInPerTick),
                                new DualNum<Time>(new double[]{
                                        (mPerpXTicks / (mPar0YTicks - mPar1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                        (mPerpXTicks / (mPar0YTicks - mPar1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                                }).times(mInPerTick)
                        ),
                        new DualNum<>(new double[]{
                                (par0PosDelta - par1PosDelta) / (mPar0YTicks - mPar1YTicks),
                                (par0PosVel.velocity - par1PosVel.velocity) / (mPar0YTicks - mPar1YTicks),
                        })
                );

            }

            mLastPar0Pos = par0PosVel.position;
            mLastPar1Pos = par1PosVel.position;
            mLastPerpPos = perpPosVel.position;

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
     * Determines if the 3 dead wheel component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /**
     * Reads and applies the 3 dead wheel configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    @Override
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        mPar0 = null;
        mPar1 = null;
        mPerp = null;

        try {

            if(mMap != null && reader.has(sPar0HwMapKey)) {
                mPar0HwMapName = reader.getString(sPar0HwMapKey);
//                DcMotorEx motor = hwMap.tryGet(DcMotorEx.class,conf.get(0).mapName());
//                if(motor != null) { mPar0 = new OverflowEncoder(new RawEncoder(motor)); }
//                if(mPar0 != null && MotorComponent.sString2Direction.containsKey(conf.get(0).direction())) {
//                    mPar0.setDirection(Objects.requireNonNull(MotorComponent.sString2Direction.get(conf.get(0).direction())));
//                }
//                else if(mPar0 != null)                {
//                    mPar0.setDirection(DcMotorSimple.Direction.FORWARD);
//                }
            }
            if(mMap != null && reader.has(sPar1HwMapKey)) {
                mPar1HwMapName = reader.getString(sPar1HwMapKey);
            }
            if(mMap != null && reader.has(sPerpHwMapKey)) {
                mPerpHwMapName = reader.getString(sPar1HwMapKey);
            }

            mPar0YTicks = 0.0;
            mPar1YTicks = 1.0;
            mPerpXTicks = 0.0;
            mInPerTick  = 1.0;

            if (reader.has(sPar0TicksKey)) {
                mPar0YTicks = reader.getDouble(sPar0TicksKey);
            }
            if (reader.has(sPar1TicksKey)) {
                mPar1YTicks = reader.getDouble(sPar1TicksKey);
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

        if(mPar0 == null) { mConfigurationValid = false; }
        if(mPar1 == null) { mConfigurationValid = false; }
        if(mPerp == null) { mConfigurationValid = false; }

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

                writer.put(sPar0HwMapKey,mPar0HwMapName);
                writer.put(sPar1HwMapKey,mPar1HwMapName);
                writer.put(sPerpHwMapKey,mPerpHwMapName);
                writer.put(sInPerTickKey,mInPerTick);
                writer.put(sPar0TicksKey,mPar0YTicks);
                writer.put(sPar1TicksKey,mPar1YTicks);
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
                    .append(" - HW : Par0 ")
                    .append(mPar0HwMapName)
                    .append(" Par1 ")
                    .append(mPar1HwMapName)
                    .append(" Perp ")
                    .append(mPerpHwMapName)
                    .append(" - IPT : ")
                    .append(mInPerTick)
                    .append(" - P0T : ")
                    .append(mPar0YTicks)
                    .append(" - P1T : ")
                    .append(mPar1YTicks)
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
                    .append("> HW : Par0 ")
                    .append(mPar0HwMapName)
                    .append(" Par1 ")
                    .append(mPar1HwMapName)
                    .append(" Perp ")
                    .append(mPerpHwMapName)
                    .append(" - IPT : ")
                    .append(mInPerTick)
                    .append(" - P0T : ")
                    .append(mPar0YTicks)
                    .append(" - P1T : ")
                    .append(mPar1YTicks)
                    .append(" - PPT : ")
                    .append(mPerpXTicks)
                    .append("\n");
        }

        return result.toString();

    }
}

/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localization using OTOS sparkfun component
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.odometers;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

/* ACME robotics includes */
import com.acmerobotics.roadrunner.ftc.OTOSKt;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

/* FTC controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;
import org.json.JSONException;
import org.json.JSONObject;


public class OpticalTrackingOdometer implements OdometerComponent {

    static  final public    String sHwMapKey          = "hwmap";
    static  final public    String sHeadingRatioKey   = "heading-ratio";
    static  final public    String sPositionRatioKey  = "position-ratio";
    static  final public    String sXOffsetKey        = "x-offset";
    static  final public    String sYOffsetKey        = "y-offset";
    static  final public    String sHeadingOffsetKey  = "heading-offset";

    final LogManager            mLogger;

    boolean                     mConfigurationValid;

    final String                mName;
    String                      mHwName;

    final HardwareMap           mMap;
    SparkFunOTOS                mOtos;

    Pose2d                      mCurrentPose;
    PoseVelocity2d              mCurrentVelocity;

    public  OpticalTrackingOdometer(String name, HardwareMap hwMap, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;
        mName               = name;

        mMap                = hwMap;

        mCurrentPose        = new Pose2d(new Vector2d(0,0),0);
        mCurrentVelocity    = new PoseVelocity2d(new Vector2d(0, 0), 0);

        mOtos               = null;
    }

    @Override
    public void                         pose(Pose2d current) {
        if(mConfigurationValid) {
            mOtos.setPosition(OTOSKt.RRPoseToOTOSPose(current));
        }
    }

    @Override
    public void                         update() {
        if(mConfigurationValid) {

            SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
            SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
            SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();

            mOtos.getPosVelAcc(otosPose,otosVel,otosAcc);
            mCurrentPose = OTOSKt.OTOSPoseToRRPose(otosPose);

            mCurrentVelocity = new PoseVelocity2d(new Vector2d(otosVel.x, otosVel.y), otosVel.h);

            Vector2d fieldVel = new Vector2d(otosVel.x, otosVel.y);
            Vector2d robotVel = fieldVel.times(otosVel.h);
            mCurrentVelocity = new PoseVelocity2d(robotVel, otosVel.h);
        }
    }
    @Override
    public Pose2d                       pose() { return mCurrentPose; }

    @Override
    public PoseVelocity2d               velocity() { return mCurrentVelocity;    }

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
     * Determines if the OTOS component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /**
     * Reads and applies the OTOS configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    @Override
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        mOtos = null;

        try {

            if (mMap != null && reader.has(sHwMapKey)) {
                mHwName = reader.getString(sHwMapKey);
                mOtos = mMap.tryGet(SparkFunOTOS.class, mHwName);
            }

            if (mOtos != null) {

                SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
                double headingRatio = 1.0;
                double positionRatio = 1.0;

                if (reader.has(sHeadingRatioKey)) {
                    headingRatio = reader.getDouble(sHeadingRatioKey);
                }
                if (reader.has(sPositionRatioKey)) {
                    positionRatio = reader.getDouble(sPositionRatioKey);
                }
                if (reader.has(sXOffsetKey)) {
                    double param = reader.getDouble(sXOffsetKey);
                    offset = new SparkFunOTOS.Pose2D(param, offset.y, offset.h);
                }
                if (reader.has(sYOffsetKey)) {
                    double param = reader.getDouble(sYOffsetKey);
                    offset = new SparkFunOTOS.Pose2D(offset.x, param, offset.h);
                }
                if (reader.has(sHeadingOffsetKey)) {
                    double param = reader.getDouble(sHeadingOffsetKey);
                    offset = new SparkFunOTOS.Pose2D(offset.x, offset.y, param);
                }

                mOtos.setLinearUnit(DistanceUnit.INCH);
                mOtos.setAngularUnit(AngleUnit.RADIANS);

                mOtos.setOffset(offset);
                mOtos.setLinearScalar(positionRatio);
                mOtos.setAngularScalar(headingRatio);

                Pose2d origin = new Pose2d(new Vector2d(0, 0), 0);

                this.pose(origin);
            }
        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

        if (mOtos == null) { mConfigurationValid = false; }

    }

    /**
     * Writes the current otos configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) {

        if(mConfigurationValid) {

            try {

                writer.put(sHwMapKey,mHwName);
                writer.put(sHeadingOffsetKey, mOtos.getOffset().h);
                writer.put(sXOffsetKey, mOtos.getOffset().x);
                writer.put(sYOffsetKey, mOtos.getOffset().y);
                writer.put(sHeadingRatioKey, mOtos.getAngularScalar());
                writer.put(sPositionRatioKey, mOtos.getLinearScalar());

            } catch (JSONException e) { mLogger.error(e.getMessage()); }
        }

    }

    /**
     * Generates an HTML representation of the otos configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted otos configuration.
     */
    @Override
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {

            result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                        .append(" - HW : ")
                        .append(mHwName)
                        .append(" - OFFSET : ")
                        .append(mOtos.getOffset().x)
                        .append(",")
                        .append(mOtos.getOffset().y)
                        .append(",")
                        .append(mOtos.getOffset().h)
                        .append(" - RATIO : ")
                        .append(mOtos.getAngularScalar())
                        .append(",")
                        .append(mOtos.getLinearScalar())
                        .append("</li>\n");

        }

        return result.toString();

    }

    /**
     * Generates a text-based representation of the otos configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted otos configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {

            result.append(header)
                    .append("> HW : ")
                    .append(mHwName)
                    .append(" - OFFSET : ")
                    .append(mOtos.getOffset().x)
                    .append(",")
                    .append(mOtos.getOffset().y)
                    .append(",")
                    .append(mOtos.getOffset().h)
                    .append(" - RATIO : ")
                    .append(mOtos.getAngularScalar())
                    .append(",")
                    .append(mOtos.getLinearScalar())
                    .append("\n");
        }

        return result.toString();

    }

    /* -------------------- Accessors for tuning ------------------- */

    public void                         headingRatio(double ratio) {
        if(mConfigurationValid) { mOtos.setAngularScalar(ratio); }
    }

    public void                         positionRatio(double ratio) {
        if(mConfigurationValid) { mOtos.setLinearScalar(ratio); }
    }

    public void                         xOffset(double offset) {
        SparkFunOTOS.Pose2D newOffset = new SparkFunOTOS.Pose2D(
                offset, mOtos.getOffset().y, mOtos.getOffset().h);
        if(mConfigurationValid) { mOtos.setOffset(newOffset); }
    }

    public void                         yOffset(double offset) {
        SparkFunOTOS.Pose2D newOffset = new SparkFunOTOS.Pose2D(
                mOtos.getOffset().x, offset, mOtos.getOffset().h);
        if(mConfigurationValid) { mOtos.setOffset(newOffset); }
    }

    public void                         headingOffset(double offset) {
        SparkFunOTOS.Pose2D newOffset = new SparkFunOTOS.Pose2D(
                mOtos.getOffset().x, mOtos.getOffset().y, offset);
        if(mConfigurationValid) { mOtos.setOffset(newOffset); }
    }

}

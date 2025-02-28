/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localization using OTOS sparkfun component
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.localizers;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

/* PedroPathing includes */
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.pathgen.MathFunctions;

/* FTC controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class LocalizerOTOS extends LocalizerComponent {

    static  final public    String sTypeKey           = "otos";
    static  final public    String sHwMapKey          = "hwmap";
    static  final public    String sHeadingRatioKey   = "heading-ratio";
    static  final public    String sPositionRatioKey  = "position-ratio";
    static  final public    String sXOffsetKey        = "x-offset";
    static  final public    String sYOffsetKey        = "y-offset";
    static  final public    String sHeadingOffsetKey  = "heading-offset";

    final LogManager                mLogger;

    protected boolean               mConfigurationValid;

    final String                    mName;
    String                          mHwName;

    final HardwareMap               mMap;
    protected SparkFunOTOS          mOtos;

    Pose                            mStartPose;
    SparkFunOTOS.Pose2D             mOtosPose;
    SparkFunOTOS.Pose2D             mOtosVelocity;
    SparkFunOTOS.Pose2D             mOtosAcceleration;
    SparkFunOTOS.Pose2D             mOffset;
    double                          mPreviousHeading;
    double                          mTotalHeading;

    /**
     * Constructor
     *
     * @param name The localizer name
     * @param map The hardare map to get sensors from
     * @param logger The logger to use for traces
     */
    public  LocalizerOTOS(String name, HardwareMap map, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;
        mName               = name;

        mMap                = map;

        mOtosPose           = new SparkFunOTOS.Pose2D();
        mOtosVelocity       = new SparkFunOTOS.Pose2D();
        mOtosAcceleration   = new SparkFunOTOS.Pose2D();

        mOtos               = null;

        mTotalHeading       = 0;
        mStartPose          = new Pose(0,0,0);
        mPreviousHeading    = mStartPose.getHeading();
    }

    /**
     * Localizer name
     * @return The name
     */
    @Override
    public String                       name() { return mName; }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        Pose pose = new Pose(mOtosPose.x, mOtosPose.y, mOtosPose.h);

        Vector vec = pose.getVector();
        vec.rotateVector(mStartPose.getHeading());

        Pose result = MathFunctions.addPoses(mStartPose, new Pose(vec.getXComponent(), vec.getYComponent(), pose.getHeading()));

        return result;
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        return new Pose(mOtosVelocity.x, mOtosVelocity.y, mOtosVelocity.h);
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return getVelocity().getVector();
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param pose the new start pose
     */
    @Override
    public void setStartPose(Pose pose) {
        if(mConfigurationValid) {
            mStartPose = pose;
        }
    }


    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param pose the new current pose estimate
     */
    @Override
    public void                         setPose(Pose pose) {
        resetOTOS();
        Pose OTOSPose = MathFunctions.subtractPoses(pose, mStartPose);
        mOtos.setPosition(new SparkFunOTOS.Pose2D(OTOSPose.getX(), OTOSPose.getY(), OTOSPose.getHeading()));
    }

    /**
     * This updates the total heading of the robot. The OTOS handles all other updates itself.
     */
    @Override
    public void                         update() {
        mOtos.getPosVelAcc(mOtosPose,mOtosVelocity,mOtosAcceleration);
        mTotalHeading += MathFunctions.getSmallestAngleDifference(mOtosPose.h, mPreviousHeading);
        mPreviousHeading = mOtosPose.h;
    }

    /**
     * This resets the OTOS.
     */
    public void                         resetOTOS() {
        mOtos.resetTracking();
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    @Override
    public double                       getTotalHeading() {
        return mTotalHeading;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from OTOS
     * ticks to inches. For the OTOS, this value is the same as the lateral multiplier.
     * This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    @Override
    public double                       getForwardMultiplier() {
        return mOtos.getLinearScalar();
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * OTOS ticks to inches. For the OTOS, this value is the same as the forward multiplier.
     * This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    @Override
    public double                       getLateralMultiplier() {
        return mOtos.getLinearScalar();
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from OTOS ticks
     * to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    @Override
    public double                       getTurningMultiplier() {
        return mOtos.getAngularScalar();
    }

    /**
     * This sets the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @param value The turning ticks to radians multiplier
     */
    public void                         setForwardMultiplier(double value) {
        if(mConfigurationValid) { mOtos.setLinearScalar(value); }
    }

    /**
     * This sets the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @param value The turning ticks to radians multiplier
     */
    public void                         setLateralMultiplier(double value) {
        if(mConfigurationValid) {  mOtos.setLinearScalar(value); }
    }

    /**
     * This sets the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @param value The turning ticks to radians multiplier
     */
    public void                         setTurningMultiplier(double value) {
        if(mConfigurationValid) { mOtos.setAngularScalar(value); }
    }

    /**
     * This does nothing since this localizer does not use the IMU.
     */
    @Override
    public void                         resetIMU() {
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns whether the robot's position is NaN
     */
    @Override
    public boolean                      isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }

    /**
     * Localization logging function
     */
    @Override
    public void                         log() {
        if (mConfigurationValid) {
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-x", mOtosPose.x + " inches");
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-y", mOtosPose.y + " inches");
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-heading", mOtosPose.h / Math.PI * 180 + " deg");

            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-vx",mOtosVelocity.x + " inches/s");
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-vy",mOtosVelocity.y + " inches/s");
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-vheading",mOtosVelocity.h / Math.PI * 180 + " deg/s");
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

                mOffset = new SparkFunOTOS.Pose2D(0, 0, 0);
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
                    mOffset = new SparkFunOTOS.Pose2D(param, mOffset.y, mOffset.h);
                }
                if (reader.has(sYOffsetKey)) {
                    double param = reader.getDouble(sYOffsetKey);
                    mOffset = new SparkFunOTOS.Pose2D(mOffset.x, param, mOffset.h);
                }
                if (reader.has(sHeadingOffsetKey)) {
                    double param = reader.getDouble(sHeadingOffsetKey);
                    mOffset = new SparkFunOTOS.Pose2D(mOffset.x, mOffset.y, param);
                }

                mOtos.calibrateImu(255,true);

                mOtos.setLinearUnit(DistanceUnit.INCH);
                mOtos.setAngularUnit(AngleUnit.RADIANS);

                mOtos.setOffset(mOffset);
                mOtos.setLinearScalar(positionRatio);
                mOtos.setAngularScalar(headingRatio);

                Pose origin = new Pose(0,0,0);

                this.setStartPose(origin);
                mPreviousHeading = mStartPose.getHeading();
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
                writer.put(sHeadingOffsetKey, mOffset.h);
                writer.put(sXOffsetKey, mOffset.x);
                writer.put(sYOffsetKey, mOffset.y);
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
                        .append(mOffset.x)
                        .append(",")
                        .append(mOffset.y)
                        .append(",")
                        .append(mOffset.h)
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
                    .append(mOffset.x)
                    .append(",")
                    .append(mOffset.y)
                    .append(",")
                    .append(mOffset.h)
                    .append(" - RATIO : ")
                    .append(mOtos.getAngularScalar())
                    .append(",")
                    .append(mOtos.getLinearScalar())
                    .append("\n");
        }

        return result.toString();

    }
}

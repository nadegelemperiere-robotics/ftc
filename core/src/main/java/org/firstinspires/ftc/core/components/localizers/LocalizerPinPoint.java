/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localization using OTOS sparkfun component
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.localizers;

/* System includes */
import java.util.Objects;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* PedroPathing includes */
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.NanoTimer;

/* FTC controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class LocalizerPinPoint extends LocalizerComponent {

    static  final public    String sTypeKey                 = "pinpoint";
    static  final public    String sHwMapKey                = "hwmap";
    static  final public    String sXStrafeKey              = "x-strafe";
    static  final public    String sYForwardKey             = "y-forward";
    static  final public    String sYawScalarKey            = "yaw-scalar";
    static  final public    String sEncoderResolutionKey    = "resolution";

    final LogManager                            mLogger;

    boolean                                     mConfigurationValid;
    boolean                                     mPinPointCooked;

    final String                                mName;
    String                                      mPinPointHwName;

    final HardwareMap                           mMap;
    GoBildaPinpointDriver                       mPinPoint;

    double                                      mPreviousHeading;
    double                                      mTotalHeading;
    Pose                                        mStartPose;
    long                                        mDeltaTimeNano;
    NanoTimer                                   mTimer;
    Pose                                        mCurrentVelocity;
    Pose                                        mPinpointPose;

    double                                      mXStrafe;
    double                                      mYForward;
    double                                      mEncoderResolution;
    double                                      mYawScalar;

    public  LocalizerPinPoint(String name, HardwareMap hwMap, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;
        mPinPointCooked     = false;

        mName               = name;
        mPinPointHwName     = "";

        mMap                = hwMap;
        mPinPoint           = null;

        setStartPose(new Pose(0,0,0));
        mTotalHeading       = 0;
        mTimer              = new NanoTimer();
        mPinpointPose       = new Pose();
        mCurrentVelocity    = new Pose();
        mDeltaTimeNano      = 1;
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
    public Pose                         getPose() {
        return mPinpointPose.copy();
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose                         getVelocity() {
        return mCurrentVelocity.copy();
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector                       getVelocityVector() {
        return mCurrentVelocity.getVector();
    }

    /**
     * This sets the start pose. Since nobody should be using this after the robot has begun moving,
     * and due to issues with the PinpointLocalizer, this is functionally the same as setPose(Pose).
     *
     * @param pose the new start pose
     */
    @Override
    public void                         setStartPose(Pose pose) {
        if (!Objects.equals(mStartPose, new Pose()) && mStartPose != null) {
            Pose currentPose = MathFunctions.subtractPoses(MathFunctions.rotatePose(mPinpointPose, -mStartPose.getHeading(), false), mStartPose);
            setPose(MathFunctions.addPoses(pose, MathFunctions.rotatePose(currentPose, pose.getHeading(), false)));
        } else {
            setPose(pose);
        }

        this.mStartPose = pose;
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param pose the new current pose estimate
     */
    @Override
    public void                         setPose(Pose pose) {
        mPinPoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading()));
        mPinpointPose = pose;
        mPreviousHeading = pose.getHeading();
    }

    /**
     * This updates the total heading of the robot. The Pinpoint handles all other updates itself.
     */
    @Override
    public void                         update() {
        mDeltaTimeNano = mTimer.getElapsedTime();
        mTimer.resetTimer();
        mPinPoint.update();
        Pose currentPinpointPose = getPoseEstimate(mPinPoint.getPosition(), mPinpointPose, mDeltaTimeNano);
        mTotalHeading += MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), mPreviousHeading);
        mPreviousHeading = currentPinpointPose.getHeading();
        Pose deltaPose = MathFunctions.subtractPoses(currentPinpointPose, mPinpointPose);
        mCurrentVelocity = new Pose(deltaPose.getX() / (mDeltaTimeNano / Math.pow(10.0, 9)), deltaPose.getY() / (mDeltaTimeNano / Math.pow(10.0, 9)), deltaPose.getHeading() / (mDeltaTimeNano / Math.pow(10.0, 9)));
        mPinpointPose = currentPinpointPose;
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
     * This returns the Y encoder value as none of the odometry tuners are required for this localizer
     * @return returns the Y encoder value
     */
    @Override
    public double                       getForwardMultiplier() {
        return mPinPoint.getEncoderY();
    }

    /**
     * This returns the X encoder value as none of the odometry tuners are required for this localizer
     * @return returns the X encoder value
     */
    @Override
    public double                       getLateralMultiplier() {
        return mPinPoint.getEncoderX();
    }

    /**
     * This returns either the factory tuned yaw scalar or the yaw scalar tuned by yourself.
     * @return returns the yaw scalar
     */
    @Override
    public double                       getTurningMultiplier() {
        return mPinPoint.getYawScalar();
    }

    /**
     * This sets the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @param value The turning ticks to radians multiplier
     */
    public void                         setForwardMultiplier(double value) {
        if(mConfigurationValid) { mPinPoint.setOffsets(mPinPoint.getEncoderX(), value); }
    }

    /**
     * This sets the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @param value The turning ticks to radians multiplier
     */
    public void                         setLateralMultiplier(double value) {
        if(mConfigurationValid) {  mPinPoint.setOffsets(value, mPinPoint.getEncoderY()); }
    }

    /**
     * This sets the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @param value The yaw scalar
     */
    public void                         setTurningMultiplier(double value) {
        if(mConfigurationValid) { mPinPoint.setYawScalar(value); }
    }

    /**
     * This sets the offsets and converts inches to millimeters
     * @param xOffset How far to the side from the center of the robot is the x-pod? Use positive values if it's to the left and negative if it's to the right.
     * @param yOffset How far forward from the center of the robot is the y-pod? Use positive values if it's forward and negative if it's to the back.
     * @param unit The units that the measurements are given in
     */
    private void                        setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        mPinPoint.setOffsets(unit.toMm(xOffset), unit.toMm(yOffset));
    }

    /**
     * This resets the IMU. Does not change heading estimation.
     */
    @Override
    public void                         resetIMU() throws InterruptedException {
        mPinPoint.recalibrateIMU();
    }

    /**
     * This resets the pinpoint.
     */
    private void                        resetPinpoint() {
        mPinPoint.resetPosAndIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private Pose                        getPoseEstimate(Pose2D pinpointEstimate, Pose currentPose, long deltaTime) {
        if (Double.isNaN(pinpointEstimate.getX(DistanceUnit.INCH)) || Double.isNaN(pinpointEstimate.getY(DistanceUnit.INCH)) || Double.isNaN(pinpointEstimate.getHeading(AngleUnit.RADIANS))) {
            mPinPointCooked = true;
            return MathFunctions.addPoses(currentPose, new Pose(mCurrentVelocity.getX() * deltaTime / Math.pow(10, 9), mCurrentVelocity.getY() * deltaTime / Math.pow(10, 9), mCurrentVelocity.getHeading() * deltaTime / Math.pow(10, 9)));
        }

        Pose estimate = new Pose(pinpointEstimate.getX(DistanceUnit.INCH), pinpointEstimate.getY(DistanceUnit.INCH), pinpointEstimate.getHeading(AngleUnit.RADIANS));

        mPinPointCooked = false;
        return estimate;
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns whether the robot's position is NaN
     */
    @Override
    public boolean                      isNAN() {
        return mPinPointCooked;
    }

    /**
     * Localization logging function
     */
    @Override
    public void                         log() {
        if (mConfigurationValid) {
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-x", mPinpointPose.getX() + " inches");
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-y", mPinpointPose.getY() + " inches");
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-heading", mPinpointPose.getHeading() / Math.PI * 180 + " deg");

            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-vx",mCurrentVelocity.getX() + " inches/s");
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-vy",mCurrentVelocity.getY() + " inches/s");
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-vheading",mCurrentVelocity.getHeading() / Math.PI * 180 + " deg/s");
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
        mPinPoint = null;

        try {

            if(mMap != null && reader.has(sHwMapKey)) {
                mPinPointHwName = reader.getString(sHwMapKey);
                mPinPoint = mMap.tryGet(GoBildaPinpointDriver.class,mPinPointHwName);
            }

            mXStrafe = 0;
            mYForward = 0;
            mYawScalar = 0;
            mEncoderResolution = 0;

            if (reader.has(sXStrafeKey)) {
                mXStrafe = reader.getDouble(sXStrafeKey);
            }
            if (reader.has(sYForwardKey)) {
                mYForward = reader.getDouble(sYForwardKey);
            }
            if (reader.has(sEncoderResolutionKey)) {
                mEncoderResolution = reader.getDouble(sEncoderResolutionKey);
            }
            if(reader.has(sYawScalarKey)) {
                mYawScalar = reader.getDouble(sYawScalarKey);
            }

            if(mPinPoint != null)  {
                this.setOffsets(mXStrafe, mYForward,DistanceUnit.INCH);
                if(mEncoderResolution == 0) { mPinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); }
                else { mPinPoint.setEncoderResolution(mEncoderResolution); }
                if(mYawScalar != 0) { mPinPoint.setYawScalar(mYawScalar); }
                this.resetPinpoint();
            }

            setStartPose(new Pose(0,0,0));
            mTotalHeading       = 0;
            mDeltaTimeNano      = 1;
            mTimer.resetTimer();
            mPinpointPose       = new Pose();
            mCurrentVelocity    = new Pose();
            mPreviousHeading    = mStartPose.getHeading();
        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

        if(mPinPoint == null) { mConfigurationValid = false; }

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

                writer.put(sHwMapKey,mPinPointHwName);
                writer.put(sXStrafeKey,mXStrafe);
                writer.put(sYForwardKey,mYForward);
                if(mEncoderResolution != 0) {
                    writer.put(sEncoderResolutionKey, mEncoderResolution);
                }
                if(mYawScalar != 0) { writer.put(sYawScalarKey, mYawScalar); }

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
                    .append(" - HW : ")
                    .append(mPinPointHwName)
                    .append(" - OFFSET : ")
                    .append(mXStrafe)
                    .append(",")
                    .append(mYForward);

            if(mEncoderResolution != 0) {
                result.append(" - RES : ")
                    .append(mEncoderResolution);
            }

            if(mYawScalar != 0) {
                result.append(" - YAW SC : ")
                        .append(mYawScalar);
            }

            result.append("</li>\n");

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
                    .append("> HW : ")
                    .append(mPinPointHwName)
                    .append(" - OFFSET : ")
                    .append(mXStrafe)
                    .append(",")
                    .append(mYForward);

            if(mEncoderResolution != 0) {
                result.append(" - RES : ")
                        .append(mEncoderResolution);
            }

            if(mYawScalar != 0) {
                result.append(" - YAW SC : ")
                        .append(mYawScalar);
            }

            result.append("\n");
        }

        return result.toString();

    }


}

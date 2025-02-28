/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Mock Localization
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.localizers;

/* JSON includes */
import org.json.JSONObject;

/* PedroPathing includes */
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.Matrix;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class LocalizerMock extends LocalizerComponent {

    static  final public String sTypeKey = "mock";

    final LogManager            mLogger;

    final boolean               mConfigurationValid;

    final String                mName;

    Pose                        mStartPose;
    Pose                        mCurrentPose;
    Pose                        mCurrentVelocity;
    double                      mTotalHeading;
    double                      mPreviousHeading;

    public LocalizerMock(String name, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = true;

        mName               = name;

        mCurrentPose     = new Pose();
        mStartPose       = new Pose();
        mCurrentVelocity = new Pose();
        mTotalHeading    = 0;
        mPreviousHeading = 0;

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
        return mCurrentPose;
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose                         getVelocity() {
        return mCurrentVelocity;
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector                       getVelocityVector() {
        return getVelocity().getVector();
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param pose the new start pose
     */
    @Override
    public void                         setStartPose(Pose pose) {
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

        mCurrentPose = pose;}

    /**
     * This updates the total heading of the robot. The OTOS handles all other updates itself.
     */
    @Override
    public void                         update() {
        mTotalHeading += MathFunctions.getSmallestAngleDifference(mCurrentPose.getHeading(), mPreviousHeading);
        mPreviousHeading = mCurrentPose.getHeading();
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
        return 1.0;
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
        return 1.0;
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from OTOS ticks
     * to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    @Override
    public double                       getTurningMultiplier() {
        return 1.0;
    }

    /**
     * This sets the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @param value The turning ticks to radians multiplier
     */
    public void                         setForwardMultiplier(double value) {}

    /**
     * This sets the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @param value The turning ticks to radians multiplier
     */
    public void                         setLateralMultiplier(double value) {}

    /**
     * This sets the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @param value The turning ticks to radians multiplier
     */
    public void                         setTurningMultiplier(double value) {}

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
    public boolean isNAN() {
        return false;
    }

    /**
     * Localization logging function
     */
    @Override
    public void             log() {
        if (mConfigurationValid) {
            mLogger.metric("x", mCurrentPose.getX() + " inches");
            mLogger.metric("y", mCurrentPose.getY() + " inches");
            mLogger.metric("heading", mCurrentPose.getHeading() / Math.PI * 180 + " deg");

            mLogger.metric("vx",mCurrentVelocity.getX() + " inches/s");
            mLogger.metric("vy",mCurrentVelocity.getY() + " inches/s");
            mLogger.metric("vheading",mCurrentVelocity.getHeading() / Math.PI * 180 + " deg/s");
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
    public void                         read(JSONObject reader) { }

    /**
     * Writes the current drive odometer configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) { }

    /**
     * Generates an HTML representation of the drive odometer configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted drive odometer configuration.
     */
    @Override
    public String                       logConfigurationHTML() { return "<p>Mock</p>\n"; }

    /**
     * Generates a text-based representation of the drive odometer configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted drive odometer configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {  return header + "> Mock\n"; }

}

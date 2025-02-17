/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localization using OTOS sparkfun component
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.odometers;

/* System includes */
import java.util.LinkedList;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* ACME robotics includes */
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;

/* FTC controller includes */
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class PinPointOdometer implements OdometerComponent {

    static  final public    String sHwMapKey                = "hwmap";
    static  final public    String sXOffsetKey              = "x-offset";
    static  final public    String sYOffsetKey              = "y-offset";
    static  final public    String sEncoderResolutionKey    = "resolution";

    static  final           double sInToMm            = 25.4;

    LogManager                  mLogger;

    boolean                     mConfigurationValid;

    String                      mName;
    String                      mPinPointHwName;

    HardwareMap                 mMap;
    GoBildaPinpointDriverRR     mPinPoint;

    Pose2d                      mInitialPose;
    Pose2d                      mCurrentPose;
    Pose2d                      mRobotPose;
    PoseVelocity2d              mCurrentVelocity;
    public LinkedList<Pose2d>   mPoseHistory;

    double                      mXOffset;
    double                      mYOffset;
    double                      mEncoderResolution;

    double                      mParYTicks;
    double                      mPerpXTicks;
    double                      mInPerTick;

    public  PinPointOdometer(String name, HardwareMap hwMap, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;

        mName           = name;
        mPinPointHwName = "";

        mPinPoint        = null;
        mRobotPose       = new Pose2d(new Vector2d(0,0),0);
        mCurrentPose     = new Pose2d(new Vector2d(0,0),0);
        mCurrentVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
        mPoseHistory     = new LinkedList<>();

    }


    @Override
    public void     pose(Pose2d current) {
        if(mConfigurationValid) {
            mPinPoint.setPosition(current);
        }
    }

    @Override
    public void     update() {
        if(mConfigurationValid) {

            mPinPoint.update();

            mCurrentPose = mPinPoint.getPositionRR();
            mCurrentVelocity = mPinPoint.getVelocityRR();

            // RR standard
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
            mLogger.metric("x",mCurrentPose.position.x + " inches");
            mLogger.metric("y",mCurrentPose.position.y + " inches");
            mLogger.metric("heading",mCurrentPose.heading.toDouble() + " rad");

            mLogger.metric("vx",mCurrentVelocity.linearVel.x + " inches/s");
            mLogger.metric("vy",mCurrentVelocity.linearVel.y + " inches/s");
            mLogger.metric("vheading",mCurrentVelocity.angVel + " rad/s");
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
                mPinPoint = mMap.tryGet(GoBildaPinpointDriverRR.class,mPinPointHwName);
            }

            mXOffset = 0;
            mYOffset = 0;
            mEncoderResolution = GoBildaPinpointDriverRR.goBILDA_SWINGARM_POD;

            if (reader.has(sXOffsetKey)) {
                mXOffset = reader.getDouble(sXOffsetKey);
            }
            if (reader.has(sYOffsetKey)) {
                mYOffset = reader.getDouble(sYOffsetKey);
            }
            if (reader.has(sEncoderResolutionKey)) {
                mEncoderResolution = reader.getDouble(sEncoderResolutionKey);
            }

            if(mPinPoint != null)  {
                mPinPoint.setOffsets(DistanceUnit.MM.fromInches(mXOffset), DistanceUnit.MM.fromInches(mYOffset));
                mPinPoint.setEncoderResolution(mEncoderResolution);
                ///pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);
                // Check how it relates with associated motor configuration before changing it.
                mPinPoint.resetPosAndIMU();
                try {
                    Thread.sleep(300);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            Pose2d origin = new Pose2d(new Vector2d(0, 0), 0);

            this.pose(origin);
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
                writer.put(sXOffsetKey,mXOffset);
                writer.put(sYOffsetKey,mYOffset);
                writer.put(sEncoderResolutionKey,mEncoderResolution);

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
                    .append(mXOffset)
                    .append(",")
                    .append(mYOffset)
                    .append(" - RES : ")
                    .append(mEncoderResolution)
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
                    .append("> HW : ")
                    .append(mPinPointHwName)
                    .append(" - OFFSET : ")
                    .append(mXOffset)
                    .append(",")
                    .append(mYOffset)
                    .append(" - RES : ")
                    .append(mEncoderResolution)
                    .append("\n");
        }

        return result.toString();

    }

}

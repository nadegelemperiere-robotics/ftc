/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localization using 3 odometry wheels
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.localizers;

/* System includes */
import java.util.Map;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* PedroPathing includes */
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.Matrix;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.NanoTimer;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.motors.MotorComponent;
import org.firstinspires.ftc.core.components.motors.EncoderComponent;

public class LocalizerThreeDeadWheels extends LocalizerComponent {

    static  final public String sTypeKey        = "3-dead-wheels";
    static  final String    sLeftHwMapKey       = "left";
    static  final String    sRightHwMapKey      = "right";
    static  final String    sStrafeHwMapKey     = "strafe";
    static  final String    sForwardTicksKey    = "forward-ticks-to-inches";
    static  final String    sStrafeTicksKey     = "strafe-ticks-to-inches";
    static  final String    sTurnTicksKey       = "turn-ticks-to-inches";
    static  final String    sLeftYOffsetKey     = "left-y-offset";
    static  final String    sRightYOffsetKey    = "right-y-offset";
    static  final String    sStrafeXOffsetKey   = "strafe-x-offset";


    final protected LogManager          mLogger;

    protected boolean                   mConfigurationValid;

    final String                        mName;
    String                              mLeftHwMapName;
    String                              mRightHwMapName;
    String                              mStrafeHwMapName;

    final HardwareMap                   mMap;
    final Map<String, MotorComponent>   mMotors;
    EncoderComponent                    mLeftEncoder;
    EncoderComponent                    mRightEncoder;
    EncoderComponent                    mStrafeEncoder;

    Pose                                mStartPose;
    Pose                                mDisplacementPose;
    Pose                                mCurrentVelocity;
    Matrix                              mPrevRotationMatrix;
    NanoTimer                           mTimer;
    long                                mDeltaTimeNano;
    Pose                                mLeftEncoderPose;
    Pose                                mRightEncoderPose;
    Pose                                mStrafeEncoderPose;
    double                              mTotalHeading;

    protected double                    mForwardTicksToInches;
    protected double                    mStrafeTicksToInches;
    protected double                    mTurnTicksToInches;

    double                              mLeftY;
    double                              mRightY;
    double                              mStrafeX;

    public  LocalizerThreeDeadWheels(String name, HardwareMap hwMap, Map<String, MotorComponent> motors, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;

        mName               = name;
        mLeftHwMapName      = "";
        mRightHwMapName     = "";
        mStrafeHwMapName    = "";

        mMap            = hwMap;
        mMotors         = motors;
        mLeftEncoder    = null;
        mRightEncoder   = null;
        mStrafeEncoder  = null;

        setStartPose(new Pose());
        mLeftEncoderPose    = new Pose();
        mRightEncoderPose   = new Pose();
        mStrafeEncoderPose  = new Pose();
        mTimer              = new NanoTimer();
        mDeltaTimeNano      = 1;
        mDisplacementPose   = new Pose();
        mCurrentVelocity    = new Pose();
        mTotalHeading       = 0;
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
        return MathFunctions.addPoses(mStartPose, mDisplacementPose);
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        return mCurrentVelocity.copy();
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return mCurrentVelocity.getVector();
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param pose the new start pose
     */
    @Override
    public void setStartPose(Pose pose) {
        mStartPose = pose;
    }

    /**
     * This sets the Matrix that contains the previous pose's heading rotation.
     *
     * @param heading the rotation of the Matrix
     */
    public void setPrevRotationMatrix(double heading) {
        mPrevRotationMatrix = new Matrix(3,3);
        mPrevRotationMatrix.set(0, 0, Math.cos(heading));
        mPrevRotationMatrix.set(0, 1, -Math.sin(heading));
        mPrevRotationMatrix.set(1, 0, Math.sin(heading));
        mPrevRotationMatrix.set(1, 1, Math.cos(heading));
        mPrevRotationMatrix.set(2, 2, 1.0);
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param pose the new current pose estimate
     */
    @Override
    public void setPose(Pose pose) {
        mDisplacementPose = MathFunctions.subtractPoses(pose, mStartPose);
        resetEncoders();
    }

    /**
     * This updates the elapsed time timer that keeps track of time between updates, as well as the
     * change position of the Encoders. Then, the robot's global change in position is calculated
     * using the pose exponential method.
     */
    @Override
    public void update() {
        mDeltaTimeNano = mTimer.getElapsedTime();
        mTimer.resetTimer();

        updateEncoders();
        Matrix robotDeltas = getRobotDeltas();
        Matrix globalDeltas;
        setPrevRotationMatrix(getPose().getHeading());

        Matrix transformation = new Matrix(3,3);
        if (Math.abs(robotDeltas.get(2, 0)) < 0.001) {
            transformation.set(0, 0, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(0, 1, -robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 0, robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 1, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(2, 2, 1.0);
        } else {
            transformation.set(0, 0, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(0, 1, (Math.cos(robotDeltas.get(2, 0)) - 1.0) / robotDeltas.get(2, 0));
            transformation.set(1, 0, (1.0 - Math.cos(robotDeltas.get(2, 0))) / robotDeltas.get(2, 0));
            transformation.set(1, 1, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(2, 2, 1.0);
        }

        globalDeltas = Matrix.multiply(Matrix.multiply(mPrevRotationMatrix, transformation), robotDeltas);

        mDisplacementPose.add(new Pose(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));
        mCurrentVelocity = new Pose(globalDeltas.get(0, 0) / (mDeltaTimeNano / Math.pow(10.0, 9)), globalDeltas.get(1, 0) / (mDeltaTimeNano / Math.pow(10.0, 9)), globalDeltas.get(2, 0) / (mDeltaTimeNano / Math.pow(10.0, 9)));

        mTotalHeading += globalDeltas.get(2, 0);
    }

    /**
     * This updates the Encoders.
     */
    public void updateEncoders() {
        mLeftEncoder.update();
        mRightEncoder.update();
        mStrafeEncoder.update();
    }

    /**
     * This resets the Encoders.
     */
    public void resetEncoders() {
        mLeftEncoder.reset();
        mRightEncoder.reset();
        mStrafeEncoder.reset();
    }

    /**
     * This calculates the change in position from the perspective of the robot using information
     * from the Encoders.
     *
     * @return returns a Matrix containing the robot relative movement.
     */
    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3,1);
        // x/forward movement
        returnMatrix.set(0,0, mForwardTicksToInches * ((mRightEncoder.getDeltaPosition() * mLeftEncoderPose.getY() - mLeftEncoder.getDeltaPosition() * mRightEncoderPose.getY()) / (mLeftEncoderPose.getY() - mRightEncoderPose.getY())));
        //y/strafe movement
        returnMatrix.set(1,0, mStrafeTicksToInches * (mStrafeEncoder.getDeltaPosition() - mStrafeEncoderPose.getX() * ((mRightEncoder.getDeltaPosition() - mLeftEncoder.getDeltaPosition()) / (mLeftEncoderPose.getY() - mRightEncoderPose.getY()))));
        // theta/turning
        returnMatrix.set(2,0, mTurnTicksToInches * ((mRightEncoder.getDeltaPosition() - mLeftEncoder.getDeltaPosition()) / (mLeftEncoderPose.getY() - mRightEncoderPose.getY())));
        return returnMatrix;
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    public double getTotalHeading() {
        return mTotalHeading;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public double getForwardMultiplier() {
        return mForwardTicksToInches;
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public double getLateralMultiplier() {
        return mStrafeTicksToInches;
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public double getTurningMultiplier() {
        return mTurnTicksToInches;
    }

    /**
     * This sets the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @param value The turning ticks to radians multiplier
     */
    public void                         setForwardMultiplier(double value) {
        if(mConfigurationValid) { mForwardTicksToInches = value; }
    }

    /**
     * This sets the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @param value The turning ticks to radians multiplier
     */
    public void                         setLateralMultiplier(double value) {
        if(mConfigurationValid) { mStrafeTicksToInches = value; }
    }

    /**
     * This sets the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @param value The turning ticks to radians multiplier
     */
    public void                         setTurningMultiplier(double value) {
        if(mConfigurationValid) { mTurnTicksToInches = value; }
    }

    /**
     * This does nothing since this localizer does not use the IMU.
     */
    @Override
    public void resetIMU() {
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns whether the robot's position is NaN
     */
    @Override
    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }

    /**
     * Localization logging function
     */
    @Override
    public void             log() {
        if (mConfigurationValid) {
            Pose current = MathFunctions.addPoses(mStartPose, mDisplacementPose);
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-x", current.getX() + " inches");
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-y", current.getY() + " inches");
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-heading", current.getHeading() / Math.PI * 180 + " deg");

            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-vx",mCurrentVelocity.getX() + " inches/s");
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-vy",mCurrentVelocity.getY() + " inches/s");
            mLogger.metric(LogManager.Target.DASHBOARD,mName + "-vheading",mCurrentVelocity.getHeading() / Math.PI * 180 + " deg/s");
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
        mLeftEncoder = null;
        mRightEncoder = null;
        mStrafeEncoder = null;

        try {

            if(reader.has(sLeftHwMapKey)) {
                mLeftHwMapName = reader.getString(sLeftHwMapKey);
                if(mMotors.containsKey(mLeftHwMapName)) {
                    mLeftEncoder = mMotors.get(mLeftHwMapName).encoder();
                }
            }
            if(reader.has(sRightHwMapKey)) {
                mRightHwMapName = reader.getString(sRightHwMapKey);
                if(mMotors.containsKey(mRightHwMapName)) {
                    mRightEncoder= mMotors.get(mRightHwMapName).encoder();
                }
            }
            if(reader.has(sStrafeHwMapKey)) {
                mStrafeHwMapName = reader.getString(sStrafeHwMapKey);
                if(mMotors.containsKey(mStrafeHwMapName)) {
                    mStrafeEncoder = mMotors.get(mStrafeHwMapName).encoder();
                }
            }
            
            mForwardTicksToInches = .001989436789;
            mStrafeTicksToInches = .001989436789;
            mTurnTicksToInches = .001989436789;
            mLeftY = 1;
            mRightY = -1;
            mStrafeX = -2.5;

            if (reader.has(sLeftYOffsetKey)) {
                mLeftY = reader.getDouble(sLeftYOffsetKey);
            }
            if (reader.has(sRightYOffsetKey)) {
                mRightY = reader.getDouble(sRightYOffsetKey);
            }
            if (reader.has(sStrafeXOffsetKey)) {
                mStrafeX = reader.getDouble(sStrafeXOffsetKey);
            }

            if(reader.has(sForwardTicksKey)) {
                mForwardTicksToInches = reader.getDouble(sForwardTicksKey);
            }
            if(reader.has(sStrafeTicksKey)) {
                mStrafeTicksToInches = reader.getDouble(sStrafeTicksKey);
            }
            if(reader.has(sTurnTicksKey)) {
                mTurnTicksToInches = reader.getDouble(sTurnTicksKey);
            }


            setStartPose(new Pose());
            mLeftEncoderPose = new Pose(0, mLeftY, 0);
            mRightEncoderPose = new Pose(0, mRightY, 0);
            mStrafeEncoderPose = new Pose(mStrafeX, 0, Math.toRadians(90));
            mDeltaTimeNano      = 1;
            mTimer.resetTimer();
            mTotalHeading       = 0;
            resetEncoders();
        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }


        if(mLeftEncoder == null || !mLeftEncoder.isConfigured()) { mConfigurationValid = false; }
        if(mRightEncoder == null || !mRightEncoder.isConfigured()) { mConfigurationValid = false; }
        if(mStrafeEncoder == null || !mStrafeEncoder.isConfigured()) { mConfigurationValid = false; }


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

                writer.put(sLeftHwMapKey,mLeftHwMapName);
                writer.put(sRightHwMapKey,mRightHwMapName);
                writer.put(sStrafeHwMapKey,mStrafeHwMapName);

                writer.put(sForwardTicksKey,mForwardTicksToInches);
                writer.put(sStrafeTicksKey,mStrafeTicksToInches);
                writer.put(sTurnTicksKey,mTurnTicksToInches);

                writer.put(sLeftYOffsetKey, mLeftY);
                writer.put(sRightYOffsetKey, mRightY);
                writer.put(sStrafeXOffsetKey, mStrafeX);

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
                    .append(" - HW : Left ")
                    .append(mLeftHwMapName)
                    .append(" Right ")
                    .append(mRightHwMapName)
                    .append(" Strafe ")
                    .append(mStrafeHwMapName)
                    .append(" - LY : ")
                    .append(mLeftY)
                    .append(" - RY : ")
                    .append(mRightY)
                    .append(" - SX : ")
                    .append(mStrafeX)
                    .append(" - FTI : ")
                    .append(mForwardTicksToInches)
                    .append(" - STI : ")
                    .append(mStrafeTicksToInches)
                    .append(" - TTI : ")
                    .append(mTurnTicksToInches)
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
                    .append(" - HW : Left ")
                    .append(mLeftHwMapName)
                    .append(" Right ")
                    .append(mRightHwMapName)
                    .append(" Strafe ")
                    .append(mStrafeHwMapName)
                    .append(" - LY : ")
                    .append(mLeftY)
                    .append(" - RY : ")
                    .append(mRightY)
                    .append(" - SX : ")
                    .append(mStrafeX)
                    .append(" - FTI : ")
                    .append(mForwardTicksToInches)
                    .append(" - STI : ")
                    .append(mStrafeTicksToInches)
                    .append(" - TTI : ")
                    .append(mTurnTicksToInches)
                    .append("\n");
        }

        return result.toString();

    }

}

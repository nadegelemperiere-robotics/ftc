/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localization using 4 wheels encoders
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.localizers;

/* System includes */
import java.util.Map;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Pedro Pathing includes */
import com.pedropathing.localization.Matrix;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.NanoTimer;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.imus.ImuComponent;
import org.firstinspires.ftc.core.components.motors.MotorComponent;
import org.firstinspires.ftc.core.components.motors.EncoderComponent;

public class LocalizerMecanumDriveEncoder extends LocalizerComponent {

    static  final public String sTypeKey         = "mecanum-drive-encoders";
    static  final String    sLeftFrontHwMapKey   = "front-left-wheel";
    static  final String    sLeftBackHwMapKey    = "back-left-wheel";
    static  final String    sRightFrontHwMapKey  = "front-right-wheel";
    static  final String    sRightBackHwMapKey   = "back-right-wheel";
    static  final String    sImuHwMapKey         = "imu";
    static  final String    sForwardTicksKey     = "forward-ticks-to-inches";
    static  final String    sStrafeTicksKey      = "strafe-ticks-to-inches";
    static  final String    sTurnTicksKey        = "turn-ticks-to-inches";
    static  final String    sRobotWidthKey       = "robot-width";
    static  final String    sRobotLengthKey      = "robot-length";

    final LogManager                    mLogger;

    protected boolean                   mConfigurationValid;
    boolean                             mIsFirstTime;

    final String                        mName;
    String                              mLeftBackHwName;
    String                              mLeftFrontHwName;
    String                              mRightBackHwName;
    String                              mRightFrontHwName;
    String                              mImuHwName;

    final HardwareMap                   mMap;
    final Map<String, MotorComponent>   mMotors;
    final Map<String, ImuComponent>     mImus;
    protected EncoderComponent          mLeftFront;
    protected EncoderComponent          mRightFront;
    protected EncoderComponent          mLeftBack;
    protected EncoderComponent          mRightBack;
    ImuComponent                        mImu;

    protected double                    mForwardTicksToInches;
    protected double                    mStrafeTicksToInches;
    protected double                    mTurnTicksToInches;
    protected double                    mRobotWidth;
    protected double                    mRobotLength;

    Pose                                mStartPose;
    Pose                                mDisplacementPose;
    Pose                                mCurrentVelocity;
    Matrix                              mPrevRotationMatrix;
    NanoTimer                           mTimer;
    long                                mDeltaTimeNano;
    double                              mTotalHeading;

    /**
     * Drive encoder based odometer for mecanum drive train
     * @param name   Name of the encoder
     * @param motors List of motors to get the wheels from
     * @param imus List of IMUs to get the builtin from
     * @param logger Logger
     */
    public LocalizerMecanumDriveEncoder(String name, HardwareMap map, Map<String, MotorComponent> motors, Map<String, ImuComponent> imus, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;
        mIsFirstTime        = true;

        mName               = name;
        mLeftFrontHwName    = "";
        mLeftBackHwName     = "";
        mRightFrontHwName   = "";
        mRightBackHwName    = "";
        mImuHwName          = "";

        mMap        = map;
        mMotors     = motors;
        mImus       = imus;
        mLeftFront  = null;
        mLeftBack   = null;
        mRightFront = null;
        mRightBack  = null;
        mImu        = null;
        

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
        return MathFunctions.addPoses(mStartPose, mDisplacementPose);
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
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void                         setStartPose(Pose setStart) {
        mStartPose = setStart;
    }

    /**
     * This sets the Matrix that contains the previous pose's heading rotation.
     *
     * @param heading the rotation of the Matrix
     */
    public void                         setPrevRotationMatrix(double heading) {
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
     * @param setPose the new current pose estimate
     */
    @Override
    public void                         setPose(Pose setPose) {
        mDisplacementPose = MathFunctions.subtractPoses(setPose, mStartPose);
        resetEncoders();
    }

    /**
     * This updates the elapsed time timer that keeps track of time between updates, as well as the
     * change position of the Encoders. Then, the robot's global change in position is calculated
     * using the pose exponential method.
     */
    @Override
    public void                         update() {
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
    public void                         updateEncoders() {
        mLeftFront.update();
        mRightFront.update();
        mLeftBack.update();
        mRightBack.update();
    }

    /**
     * This resets the Encoders.
     */
    public void                         resetEncoders() {
        mLeftFront.reset();
        mRightFront.reset();
        mLeftBack.reset();
        mRightBack.reset();
    }

    /**
     * This calculates the change in position from the perspective of the robot using information
     * from the Encoders.
     *
     * @return returns a Matrix containing the robot relative movement.
     */
    public Matrix                       getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3,1);
        // x/forward movement
        returnMatrix.set(0,0, mForwardTicksToInches * (mLeftFront.getDeltaPosition() + mRightFront.getDeltaPosition() + mLeftBack.getDeltaPosition() + mRightBack.getDeltaPosition()));
        //y/strafe movement
        returnMatrix.set(1,0, mStrafeTicksToInches * (-mLeftFront.getDeltaPosition() + mRightFront.getDeltaPosition() + mLeftBack.getDeltaPosition() - mRightBack.getDeltaPosition()));
        // theta/turning
        returnMatrix.set(2,0, mTurnTicksToInches * ((-mLeftFront.getDeltaPosition() + mRightFront.getDeltaPosition() - mLeftBack.getDeltaPosition() + mRightBack.getDeltaPosition()) / (mRobotWidth + mRobotLength)));
        return returnMatrix;
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    public double                       getTotalHeading() {
        return mTotalHeading;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public double                       getForwardMultiplier() {
        return mForwardTicksToInches;
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public double                       getLateralMultiplier() {
        return mStrafeTicksToInches;
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public double                       getTurningMultiplier() {
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
    public void                         resetIMU() {
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns whether the robot's position is NaN
     */
    public boolean                      isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }

    /**
     * Current localization data log function
     */
    @Override
    public void                         log() {
        if (mConfigurationValid) {

            Pose current =  MathFunctions.addPoses(mStartPose, mDisplacementPose);

            mLogger.metric(LogManager.Target.DASHBOARD,"x", current.getX() + " inches");
            mLogger.metric(LogManager.Target.DASHBOARD,"y", current.getY() + " inches");
            mLogger.metric(LogManager.Target.DASHBOARD,"heading", current.getHeading() / Math.PI * 180 + " deg");

            mLogger.metric(LogManager.Target.DASHBOARD,"vx",mCurrentVelocity.getX() + " inches/s");
            mLogger.metric(LogManager.Target.DASHBOARD,"vy",mCurrentVelocity.getY() + " inches/s");
            mLogger.metric(LogManager.Target.DASHBOARD,"vheading",mCurrentVelocity.getHeading()  / Math.PI * 180 + " deg/s");
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
                if(mMotors.containsKey(mLeftBackHwName)) {
                    mLeftBack = mMotors.get(mLeftBackHwName).encoder();
                }
            }
            if(reader.has(sLeftFrontHwMapKey)) {
                mLeftFrontHwName = reader.getString(sLeftFrontHwMapKey);
                if(mMotors.containsKey(mLeftFrontHwName)) {
                    mLeftFront = mMotors.get(mLeftFrontHwName).encoder();
                }
            }
            if(reader.has(sRightBackHwMapKey)) {
                mRightBackHwName = reader.getString(sRightBackHwMapKey);
                if(mMotors.containsKey(mRightBackHwName)) {
                    mRightBack = mMotors.get(mRightBackHwName).encoder();
                }
            }
            if(reader.has(sRightFrontHwMapKey)) {
                mRightFrontHwName = reader.getString(sRightFrontHwMapKey);
                if(mMotors.containsKey(mRightFrontHwName)) {
                    mRightFront =mMotors.get(mRightFrontHwName).encoder();
                }
            }
            if(reader.has(sImuHwMapKey)) {
                mImuHwName = reader.getString(sImuHwMapKey);
                if(mImus.containsKey(mImuHwName)) {
                    mImu = mImus.get(mImuHwName);
                }
            }

            mForwardTicksToInches   = 1.0;
            mStrafeTicksToInches    = 1.0;
            mTurnTicksToInches      = 1.0;
            mRobotLength            = 1.0;
            mRobotWidth             = 1.0;

            if (reader.has(sForwardTicksKey)) {
                mForwardTicksToInches = reader.getDouble(sForwardTicksKey);
            }
            if (reader.has(sStrafeTicksKey)) {
                mStrafeTicksToInches = reader.getDouble(sStrafeTicksKey);
            }
            if (reader.has(sTurnTicksKey)) {
                mTurnTicksToInches = reader.getDouble(sTurnTicksKey);
            }
            if(reader.has(sRobotWidthKey)) {
                mRobotWidth = reader.getDouble(sRobotWidthKey);
            }
            if(reader.has(sRobotLengthKey)) {
                mRobotLength = reader.getDouble(sRobotLengthKey);
            }

            setStartPose(new Pose());
            mTimer.resetTimer();
            mDeltaTimeNano = 1;
            mDisplacementPose = new Pose();
            mCurrentVelocity = new Pose();

        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

        if(mLeftFront == null || !mLeftFront.isConfigured())    { mConfigurationValid = false; }
        if(mLeftBack == null || !mLeftBack.isConfigured())      { mConfigurationValid = false; }
        if(mRightFront == null || !mRightFront.isConfigured())  { mConfigurationValid = false; }
        if(mRightBack == null || !mRightBack.isConfigured())    { mConfigurationValid = false; }
        if(mImu == null || !mImu.isConfigured())                { mConfigurationValid = false; }

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
                writer.put(sForwardTicksKey,mForwardTicksToInches);
                writer.put(sStrafeTicksKey,mStrafeTicksToInches);
                writer.put(sTurnTicksKey,mTurnTicksToInches);
                writer.put(sRobotWidthKey, mRobotWidth);
                writer.put(sRobotLengthKey, mRobotLength);

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
                    .append(" - FTI : ")
                    .append(mForwardTicksToInches)
                    .append(" - STI : ")
                    .append(mStrafeTicksToInches)
                    .append(" - TTI : ")
                    .append(mTurnTicksToInches)
                    .append(" - RW : ")
                    .append(mRobotWidth)
                    .append(" - RL : ")
                    .append(mRobotLength)
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
                    .append(" - FTI : ")
                    .append(mForwardTicksToInches)
                    .append(" - STI : ")
                    .append(mStrafeTicksToInches)
                    .append(" - TTI : ")
                    .append(mTurnTicksToInches)
                    .append(" - RW : ")
                    .append(mRobotWidth)
                    .append(" - RL : ")
                    .append(mRobotLength)
                    .append("\n");
        }

        return result.toString();

    }
}

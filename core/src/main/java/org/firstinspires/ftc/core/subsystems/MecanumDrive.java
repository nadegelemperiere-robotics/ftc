/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Mecanum Drive management
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.subsystems;

/* System includes */
import java.util.Arrays;
import java.util.Map;
import java.util.ArrayList;
import java.util.List;

/* JSON object */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/* Pedro Pathing includes */
import static com.pedropathing.follower.FollowerConstants.automaticHoldEnd;
import static com.pedropathing.follower.FollowerConstants.cacheInvalidateSeconds;
import static com.pedropathing.follower.FollowerConstants.centripetalScaling;
import static com.pedropathing.follower.FollowerConstants.xMovement;
import static com.pedropathing.follower.FollowerConstants.yMovement;
import static com.pedropathing.follower.FollowerConstants.drivePIDFFeedForward;
import static com.pedropathing.follower.FollowerConstants.drivePIDFSwitch;
import static com.pedropathing.follower.FollowerConstants.forwardZeroPowerAcceleration;
import static com.pedropathing.follower.FollowerConstants.headingPIDFFeedForward;
import static com.pedropathing.follower.FollowerConstants.headingPIDFSwitch;
import static com.pedropathing.follower.FollowerConstants.lateralZeroPowerAcceleration;
import static com.pedropathing.follower.FollowerConstants.nominalVoltage;
import static com.pedropathing.follower.FollowerConstants.secondaryDrivePIDFFeedForward;
import static com.pedropathing.follower.FollowerConstants.secondaryHeadingPIDFFeedForward;
import static com.pedropathing.follower.FollowerConstants.secondaryTranslationalPIDFFeedForward;
import static com.pedropathing.follower.FollowerConstants.translationalPIDFFeedForward;
import static com.pedropathing.follower.FollowerConstants.translationalPIDFSwitch;
import static com.pedropathing.follower.FollowerConstants.useSecondaryDrivePID;
import static com.pedropathing.follower.FollowerConstants.useSecondaryHeadingPID;
import static com.pedropathing.follower.FollowerConstants.useSecondaryTranslationalPID;
import static com.pedropathing.follower.FollowerConstants.useVoltageCompensationInAuto;
import com.pedropathing.follower.DriveVectorScaler;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathCallback;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.FilteredPIDFController;
import com.pedropathing.util.KalmanFilter;
import com.pedropathing.util.PIDFController;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.KalmanFilterParameters;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.motors.MotorComponent;
import org.firstinspires.ftc.core.components.localizers.LocalizerComponent;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.InterOpMode;

public class MecanumDrive extends Follower implements DriveTrain {

    static final public String      sTypeKey                        = "mecanum-drive";
    static final public String      sFrontLeftKey                   = "front-left-wheel";
    static final String             sBackLeftKey                    = "back-left-wheel";
    static final String             sFrontRightKey                  = "front-right-wheel";
    static final String             sBackRightKey                   = "back-right-wheel";
    static final String             sLocalizerKey                   = "localizer";
    static final String             sMotorsKey                      = "motors";
    static final String             sFollowerKey                    = "follower";

    static final String             sReferenceKey                   = "reference";
    static final String             sRobotCentricKey                = "robot-centric";
    static final String             sFieldCentricKey                = "field-centric";
    static final String             sShortNameKey                   = "short";

    static final String             sMotorCachingThresholdKey       = "motor-caching-threshold";
    static final String             sXMovementKey                   = "x-movement";
    static final String             sYMovementKey                   = "y-movement";
    static final String             sMaxPowerKey                    = "max-power";
    static final String             sPidfPKey                       = "p";
    static final String             sPidfIKey                       = "i";
    static final String             sPidfDKey                       = "d";
    static final String             sPidfFKey                       = "f";
    static final String             sPidfTKey                       = "t";
    static final String             sTranslationPidfKey             = "translation-pidf";
    static final String             sTranslationIntegralKey         = "translation-integral";
    static final String             sTranslationPidfFFKey           = "translation-pidf-feed-forward";
    static final String             sHeadingPidfKey                 = "heading-pidf";
    static final String             sHeadingPidfFFKey               = "heading-pidf-feed-forward";
    static final String             sDrivePidfKey                   = "drive-pidf";
    static final String             sDrivePidfFFKey                 = "drive-pidf-feed-forward";
    static final String             sDriveKalmanFilterKey           = "drive-kalman";
    static final String             sKalmanModelKey                 = "model";
    static final String             sKalmanDataKey                  = "data";
    static final String             sMassKey                        = "mass";
    static final String             sCentripetalScalingKey          = "centripetal-scaling";
    static final String             sForward0PowerAccKey            = "forward-0-power-acceleration";
    static final String             sStrafe0PowerAccKey             = "strafe-0-power-acceleration";
    static final String             s0PowerAccMultiplierKey         = "0-power-acceleration-multiplier";
    static final String             sPathEndVelocityConstKey        = "path-end-velocity-constraint";
    static final String             sPathEndTranslationConstKey     = "path-end-translation-constraint";
    static final String             sPathEndHeadingConstKey         = "path-end-heading-constraint";
    static final String             sPathEndValueConstKey           = "path-end-value-constraint";
    static final String             sPathEndTimeOutConstKey         = "path-end-timeout-constraint";
    static final String             sApproximationSteps             = "approximation-steps";
    static final String             sHoldPointTranslationScalingKey = "hold-point-translation-scaling";
    static final String             sHoldPointHeadingScalingKey     = "hold-point-heading-scaling";
    static final String             sAverageVelocitySampleKey       = "average-velocity-samples-number";
    static final String             sBezierCurveSearchLimitKey      = "bezier-curve-search-limit";
    static final String             sTranslationPidfSwitch          = "translation-pidf-switch";
    static final String             sSecondTranslationPidfKey       = "second-translation-pidf";
    static final String             sSecondTranslationIntegralKey   = "second-translation-integral";
    static final String             sSecondTranslationPidfFFKey     = "second-translation-pidf-feed-forward";
    static final String             sHeadingPidfSwitch              = "heading-pidf-switch";
    static final String             sSecondHeadingPidfKey           = "second-heading-pidf";
    static final String             sSecondHeadingPidfFFKey         = "second-heading-pidf-feed-forward";
    static final String             sDrivePidfSwitch                = "drive-pidf-switch";
    static final String             sSecondDrivePidfKey             = "second-drive-pidf";
    static final String             sSecondDrivePidfFFKey           = "second-drive-pidf-feed-forward";
    static final String             sShallHoldAtEndKey              = "shall-hold-at-end";
    static final String             sShallUSeVoltageCompensationKey = "shall-use-voltage-compensation";
    static final String             sNominalVoltageKey              = "nominal-voltage";
    static final String             sCacheInvalidateSecondsKey      = "cache-invalidate-seconds";

    final LogManager                mLogger;

    protected boolean               mConfigurationValid;

    final String                    mName;
    String                          mShortName;
    String                          mLeftFrontHwName;
    String                          mLeftBackHwName;
    String                          mRightFrontHwName;
    String                          mRightBackHwName;
    String                          mLocalizerHwName;

    final Hardware                  mHardware;
    protected MotorComponent        mLeftFront;
    protected MotorComponent        mRightFront;
    protected MotorComponent        mLeftBack;
    protected MotorComponent        mRightBack;
    protected LocalizerComponent    mLocalizer;
    List<MotorComponent>            mMotors;
    VoltageSensor                   mVoltageSensor;

    double                          mDrivingSpeedMultiplier;
    Mode                            mDrivingMode;

    DriveVectorScaler               mDriveVectorScaler;

    PoseUpdater                     mPoseUpdater;
    DashboardPoseTracker            mDashboardPoseTracker;

    Pose                            mClosestPose;

    Path                            mCurrentPath;
    PathChain                       mCurrentPathChain;
    int                             mChainIndex;
    long[]                          mPathStartTimes;

    boolean                         mFollowingPathChain;
    boolean                         mHoldingPosition;
    boolean                         mIsBusy;
    boolean                         mReachedParametricPathEnd;
    boolean                         mHoldPositionAtEnd;
    boolean                         mTeleopDrive;

    double                          mGlobalMaxPower;
    double                          mPreviousSecondaryTranslationalIntegral;
    double                          mPreviousTranslationalIntegral;
    double                          mDriveError;
    double                          mHeadingError;

    long                            mReachedParametricPathEndTime;

    double[]                        mDrivePowers;
    double[]                        mTeleopDriveValues;

    ArrayList<Vector>               mVelocities;
    ArrayList<Vector>               mAccelerations;

    Vector                          mAverageVelocity;
    Vector                          mAveragePreviousVelocity;
    Vector                          mAverageAcceleration;
    Vector                          mSecondaryTranslationalIntegralVector;
    Vector                          mTranslationalIntegralVector;
    Vector                          mTeleopDriveVector;
    Vector                          mTeleopHeadingVector;
    Vector                          mDriveVector;
    Vector                          mHeadingVector;
    Vector                          mTranslationalVector;
    Vector                          mCentripetalVector;
    Vector                          mCorrectiveVector;
    double                          mCentripetalScaling;

    PIDFController                  mSecondaryTranslationalPIDF;
    PIDFController                  mSecondaryTranslationalIntegral;
    PIDFController                  mTranslationalPIDF;
    PIDFController                  mTranslationalIntegral;
    PIDFController                  mSecondaryHeadingPIDF;
    PIDFController                  mHeadingPIDF;
    FilteredPIDFController          mSecondaryDrivePIDF;
    FilteredPIDFController          mDrivePIDF;

    KalmanFilter                    mDriveKalmanFilter;
    double[]                        mDriveErrors;
    double                          mRawDriveError;
    double                          mPreviousRawDriveError;

    boolean                         mDrawOnDashboard;
    boolean                         mUseTranslational;
    boolean                         mUseCentripetal;
    boolean                         mUseHeading;
    boolean                         mUseDrive;

    /*
     * Voltage Compensation
     * Credit to team 14343 Escape Velocity for the mVoltage code
     * Credit to team 23511 Seattle Solvers for implementing the mVoltage code into Follower.java
     */
    boolean                         mCached;

    public double                   mVoltage;
    final ElapsedTime               mVoltageTimer;

    ElapsedTime                     mZeroVelocityDetectedTimer;

    /**
     * Constructor
     * @param name Name of the drive train
     * @param hardware List of registered hardware to use
     * @param logger Logger for trace
     */
    public  MecanumDrive(String name, Hardware hardware, LogManager logger) {
        super(null);

        mLogger                 = logger;
        mConfigurationValid     = false;

        mDrivingMode            = Mode.ROBOT_CENTRIC;
        mDrivingSpeedMultiplier = 1.0;

        mName               = name;
        mShortName          = "";
        mLeftFrontHwName    = "";
        mLeftBackHwName     = "";
        mRightFrontHwName   = "";
        mRightBackHwName    = "";
        mLocalizerHwName    = "";

        mHardware           = hardware;
        mLeftFront          = null;
        mRightFront         = null;
        mLeftBack           = null;
        mRightBack          = null;
        mLocalizer          = null;
        if(mHardware != null) { mVoltageSensor = mHardware.voltageSensor(); }
        
        mMotors             = new ArrayList<>();
        mVelocities         = new ArrayList<>();
        mAccelerations      = new ArrayList<>();
        mVoltageTimer       = new ElapsedTime();
        
    }

    public void                         start() {
        startTeleopDrive();
    }

    /**
     * Position reset function
     * @param pose Current position
     */
    public void                         initialize(Pose pose) {
        if(mConfigurationValid) {
            setStartingPose(pose);
        }
    }

    /**
     * Current task status
     * @return true if the train is available, false if busy
     */
    public boolean                      hasFinished() { return !this.isBusy(); }

    /**
     * Change the power multiplier when driving
     * @param multiplier a small number for precision, a greater for speed
     */
    public void                         driveSpeedMultiplier(double multiplier) {
        if(mConfigurationValid) { mDrivingSpeedMultiplier = multiplier; }
    }


    /**
     * Change the motor speed according to controller command
     * @param xSpeed x direction speed for field centric, forward for robot centric
     * @param ySpeed y direction speed for field centric, lateral for robot centric
     * @param headingSpeed rotation speed
     */
    @Override
    public void                         drive(double xSpeed, double ySpeed, double headingSpeed) {

        if(mConfigurationValid) {

            mLogger.debug(LogManager.Target.FILE,"start");

            if (mDrivingMode == Mode.FIELD_CENTRIC) {
                setTeleOpMovementVectors(
                        xSpeed* mDrivingSpeedMultiplier,
                        ySpeed* mDrivingSpeedMultiplier,
                        headingSpeed* mDrivingSpeedMultiplier,
                        false);
            }
            else if (mDrivingMode == Mode.ROBOT_CENTRIC){
                setTeleOpMovementVectors(
                        xSpeed* mDrivingSpeedMultiplier,
                        ySpeed* mDrivingSpeedMultiplier,
                        headingSpeed* mDrivingSpeedMultiplier,
                        true);
            }

            mLogger.debug(LogManager.Target.FILE,"stop");

        }
    }

    /**
     * Persist data to be able to keep the same behavior after a reinitialization.
     * Read current heading and transform it into the FTC field coordinate system
     */
    public void                         persist()
    {
        if(mConfigurationValid) {
            Pose current = mPoseUpdater.getPose();
            InterOpMode.instance().add(mName + "-pose", current);
        }
    }

    /**
     * Current state logging function
     */
    public void                         log() {
        if(mConfigurationValid) {

            mLocalizer.log();

            mLogger.info(mShortName + " POS : " +
                    " x : " + (double)((int)(mLocalizer.getPose().getX() * 100)) / 100 +
                    " - y : " + (double)((int)(mLocalizer.getPose().getY() * 100)) / 100 +
                    " - heading : " + (int)(mLocalizer.getPose().getHeading() / Math.PI * 180) + " deg");
            mLogger.info(mShortName + " SPD : " +
                    " x : " + (double)((int)(mLocalizer.getVelocity().getX()) *1000) / 1000 +
                    " - y : " + (double)((int)(mLocalizer.getVelocity().getY())*1000) / 1000 +
                    " - heading : " + (double)((int)(mLocalizer.getVelocity().getHeading() / Math.PI * 1800))/1000 + " deg/s");
        }
    }


    /**
     * This initializes the follower.
     * In this, the DriveVectorScaler and PoseUpdater is instantiated, the drive motors are
     * initialized and their behavior is set, and the variables involved in approximating first and
     * second derivatives for teleop are set.
     */

    @Override
    public void                         initialize() {

        if(mConfigurationValid) {

            mGlobalMaxPower = 1;
            mVoltage = 0;
            mCentripetalScaling = centripetalScaling;

            mVoltageTimer.reset();

            mDriveKalmanFilter              = new KalmanFilter(FollowerConstants.driveKalmanFilterParameters);
            mSecondaryTranslationalPIDF     = new PIDFController(FollowerConstants.secondaryTranslationalPIDFCoefficients);
            mSecondaryTranslationalIntegral = new PIDFController(FollowerConstants.secondaryTranslationalIntegral);
            mTranslationalPIDF              = new PIDFController(FollowerConstants.translationalPIDFCoefficients);
            mTranslationalIntegral          = new PIDFController(FollowerConstants.translationalIntegral);
            mSecondaryHeadingPIDF           = new PIDFController(FollowerConstants.secondaryHeadingPIDFCoefficients);
            mHeadingPIDF                    = new PIDFController(FollowerConstants.headingPIDFCoefficients);
            mSecondaryDrivePIDF             = new FilteredPIDFController(FollowerConstants.secondaryDrivePIDFCoefficients);
            mDrivePIDF                      = new FilteredPIDFController(FollowerConstants.drivePIDFCoefficients);

            mDrawOnDashboard    = true;
            mUseTranslational   = true;
            mUseCentripetal     = true;
            mUseHeading         = true;
            mUseDrive           = true;
            mCached             = false;

            double[] convertToPolar = Point.cartesianToPolar(xMovement, -yMovement);
            Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0], convertToPolar[1]));

            mPoseUpdater        = new PoseUpdater(null, mLocalizer);
            mDriveVectorScaler  = new DriveVectorScaler(frontLeftVector);

            // Order to match DriveVectorScaler
            mMotors.add(mLeftFront);
            mMotors.add(mLeftBack);
            mMotors.add(mRightFront);
            mMotors.add(mRightBack);
            for(int i_motor = 0; i_motor < mMotors.size(); i_motor ++) {
                mMotors.get(i_motor).achieveableMaxRPMFraction(1.0);
            }

            setMotorsToFloat();

            mDashboardPoseTracker = new DashboardPoseTracker(mPoseUpdater);

            breakFollowing();
        }
    }

    @Override
    public void setCentripetalScaling(double value) {
        mCentripetalScaling = value;
    }

    /**
     * This sets the motors to the zero power behavior of brake.
     */
    private void setMotorsToBrake() {
        if(mConfigurationValid) {
            for(int i_motor = 0; i_motor < mMotors.size(); i_motor ++) {
                mMotors.get(i_motor).zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }

    /**
     * This sets the motors to the zero power behavior of float.
     */
    private void setMotorsToFloat() {
        if(mConfigurationValid) {
            for(int i_motor = 0; i_motor < mMotors.size(); i_motor ++) {
                mMotors.get(i_motor).zeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }
    }

    /**
     * This sets the maximum power the motors are allowed to use.
     *
     * @param value This caps the motor power from [0, 1].
     */
    @Override
    public void setMaxPower(double value) {
        if(mConfigurationValid) {
            mGlobalMaxPower = value;
            mDriveVectorScaler.setMaxPowerScaling(value);
        }
    }   

    /**
     * This gets a Point from the current Path from a specified t-value.
     *
     * @return returns the Point.
     */
    @Override
    public Point getPointFromPath(double t) {
        Point result = null;
        if(mConfigurationValid && mCurrentPath != null) { result = mCurrentPath.getPoint(t); } 
        return result;
    }

    /**
     * This returns the current pose from the PoseUpdater.
     *
     * @return returns the pose
     */
    @Override
    public Pose getPose() {
        return mPoseUpdater.getPose();
    }

    /**
     * This sets the current pose in the PoseUpdater without using offsets.
     *
     * @param pose The pose to set the current pose to.
     */
    @Override
    public void setPose(Pose pose) {
        mPoseUpdater.setPose(pose);
    }

    /**
     * This returns the current velocity of the robot as a Vector.
     *
     * @return returns the current velocity as a Vector.
     */
    @Override
    public Vector getVelocity() {
        return mPoseUpdater.getVelocity();
    }

    /**
     * This returns the current acceleration of the robot as a Vector.
     *
     * @return returns the current acceleration as a Vector.
     */
    @Override
    public Vector getAcceleration() {
        return mPoseUpdater.getAcceleration();
    }

    /**
     * This returns the magnitude of the current velocity. For when you only need the magnitude.
     *
     * @return returns the magnitude of the current velocity.
     */
    @Override
    public double getVelocityMagnitude() {
        return mPoseUpdater.getVelocity().getMagnitude();
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param pose the pose to set the starting pose to.
     */
    @Override
    public void setStartingPose(Pose pose) {
        mPoseUpdater.setStartingPose(pose);
    }

    /**
     * This sets the current pose, using offsets so no reset time delay. This is better than the
     * Road Runner reset, in general. Think of using offsets as setting trim in an aircraft. This can
     * be reset as well, so beware of using the resetOffset() method.
     *
     * @param set The pose to set the current pose to.
     */
    @Override
    public void setCurrentPoseWithOffset(Pose set) {
        mPoseUpdater.setCurrentPoseWithOffset(set);
    }

    /**
     * This sets the offset for only the x position.
     *
     * @param xOffset This sets the offset.
     */
    @Override
    public void setXOffset(double xOffset) {
        mPoseUpdater.setXOffset(xOffset);
    }

    /**
     * This sets the offset for only the y position.
     *
     * @param yOffset This sets the offset.
     */
    @Override
    public void setYOffset(double yOffset) {
        mPoseUpdater.setYOffset(yOffset);
    }

    /**
     * This sets the offset for only the heading.
     *
     * @param headingOffset This sets the offset.
     */
    @Override
    public void setHeadingOffset(double headingOffset) {
        mPoseUpdater.setHeadingOffset(headingOffset);
    }

    /**
     * This returns the x offset.
     *
     * @return returns the x offset.
     */
    @Override
    public double getXOffset() {
        return mPoseUpdater.getXOffset();
    }

    /**
     * This returns the y offset.
     *
     * @return returns the y offset.
     */
    @Override
    public double getYOffset() {
        return mPoseUpdater.getYOffset();
    }

    /**
     * This returns the heading offset.
     *
     * @return returns the heading offset.
     */
    @Override
    public double getHeadingOffset() {
        return mPoseUpdater.getHeadingOffset();
    }

    /**
     * This resets all offsets set to the PoseUpdater. If you have reset your pose using the
     * setCurrentPoseUsingOffset(Pose set) method, then your pose will be returned to what the
     * PoseUpdater thinks your pose would be, not the pose you reset to.
     */
    @Override
    public void resetOffset() {
        mPoseUpdater.resetOffset();
    }

    /**
     * This holds a Point.
     *
     * @param point   the Point to stay at.
     * @param heading the heading to face.
     */
    @Override
    public void holdPoint(BezierPoint point, double heading) {
        breakFollowing();
        mHoldingPosition    = true;
        mIsBusy             = false;
        mFollowingPathChain = false;
        mCurrentPath        = new Path(point);
        mCurrentPath.setConstantHeadingInterpolation(heading);
        mClosestPose        = mCurrentPath.getClosestPoint(mPoseUpdater.getPose(), 1);
    }

    /**
     * This holds a Point.
     *
     * @param point   the Point to stay at.
     * @param heading the heading to face.
     */
    @Override
    public void holdPoint(Point point, double heading) {
        holdPoint(new BezierPoint(point), heading);
    }

    /**
     * This holds a Point.
     *
     * @param pose the Point (as a Pose) to stay at.
     */
    @Override
    public void holdPoint(Pose pose) {
        holdPoint(new Point(pose), pose.getHeading());
    }

    /**
     * This follows a Path.
     * This also makes the Follower hold the last Point on the Path.
     *
     * @param path the Path to follow.
     * @param holdEnd this makes the Follower hold the last Point on the Path.
     */
    @Override
    public void followPath(Path path, boolean holdEnd) {
        mDriveVectorScaler.setMaxPowerScaling(mGlobalMaxPower);
        breakFollowing();
        mHoldPositionAtEnd  = holdEnd;
        mIsBusy             = true;
        mFollowingPathChain = false;
        mCurrentPath        = path;
        mClosestPose        = mCurrentPath.getClosestPoint(mPoseUpdater.getPose(), FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT);
    }

    /**
     * This follows a Path.
     *
     * @param path the Path to follow.
     */
    @Override
    public void followPath(Path path) {
        followPath(path, automaticHoldEnd);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     * This also makes the Follower hold the last Point on the PathChain.
     *
     * @param pathChain the PathChain to follow.
     * @param holdEnd this makes the Follower hold the last Point on the PathChain.
     */
    @Override
    public void followPath(PathChain pathChain, boolean holdEnd) {
        followPath(pathChain, mGlobalMaxPower, holdEnd);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     *
     * @param pathChain the PathChain to follow.
     */
    @Override
    public void followPath(PathChain pathChain) {
        followPath(pathChain, automaticHoldEnd);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     * This also makes the Follower hold the last Point on the PathChain.
     *
     * @param pathChain the PathChain to follow.
     * @param maxPower the max power of the Follower for this path
     * @param holdEnd this makes the Follower hold the last Point on the PathChain.
     */
    @Override
    public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
        mDriveVectorScaler.setMaxPowerScaling(maxPower);
        breakFollowing();
        mHoldPositionAtEnd  = holdEnd;
        mPathStartTimes     = new long[pathChain.size()];
        mPathStartTimes[0]  = System.currentTimeMillis();
        mIsBusy             = true;
        mFollowingPathChain = true;
        mChainIndex         = 0;
        mCurrentPathChain   = pathChain;
        mCurrentPath        = pathChain.getPath(mChainIndex);
        mClosestPose        = mCurrentPath.getClosestPoint(mPoseUpdater.getPose(), FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT);
        mCurrentPathChain.resetCallbacks();
    }

    /**
     * Resumes pathing
     */
    @Override
    public void resumePathFollowing() {
        mPathStartTimes     = new long[mCurrentPathChain.size()];
        mPathStartTimes[0]  = System.currentTimeMillis();
        mIsBusy             = true;
        mClosestPose        = mCurrentPath.getClosestPoint(mPoseUpdater.getPose(), FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT);
    }

    /**
     * This starts teleop drive control.
     */
    @Override
    public void startTeleopDrive() {
        breakFollowing();
        mTeleopDrive = true;

        if(FollowerConstants.useBrakeModeInTeleOp) { setMotorsToBrake(); }
    }

    /**
     * Calls an update to the PoseUpdater, which updates the robot's current position estimate.
     */
    public void updatePose() {
        mPoseUpdater.update();

        if (mDrawOnDashboard) { mDashboardPoseTracker.update(); }
    }

    /**
     * This calls an update to the PoseUpdater, which updates the robot's current position estimate.
     * This also updates all the Follower's PIDFs, which updates the motor powers.
     */
    @Override
    public void update() {
        mLogger.debug(mName + " start");

        updatePose();

        if (!mTeleopDrive) {
            if (mCurrentPath != null) {
                if (mHoldingPosition) {
                    mClosestPose = mCurrentPath.getClosestPoint(mPoseUpdater.getPose(), 1);

                    mDrivePowers = mDriveVectorScaler.getDrivePowers(MathFunctions.scalarMultiplyVector(getTranslationalCorrection(), FollowerConstants.holdPointTranslationalScaling), MathFunctions.scalarMultiplyVector(getHeadingVector(), FollowerConstants.holdPointHeadingScaling), new Vector(), mPoseUpdater.getPose().getHeading());

                    for(int i_motor = 0; i_motor < mMotors.size(); i_motor ++) {
                        if (Math.abs(mMotors.get(i_motor).power() - mDrivePowers[i_motor]) > FollowerConstants.motorCachingThreshold) {
                            double mVoltageNormalized = getVoltageNormalized();

                            if (useVoltageCompensationInAuto) {
                                mLogger.trace("Power " + mDrivePowers[i_motor] * mVoltageNormalized + " to motor " + mMotors.get(i_motor).name());
                                mMotors.get(i_motor).power(mDrivePowers[i_motor] * mVoltageNormalized);
                            } else {
                                mLogger.trace("Power " + mDrivePowers[i_motor] + " to motor " + mMotors.get(i_motor).name());
                                mMotors.get(i_motor).power(mDrivePowers[i_motor]);
                            }
                        }
                        mMotors.get(i_motor).zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                    
                } else {
                    if (mIsBusy) {
                        mClosestPose = mCurrentPath.getClosestPoint(mPoseUpdater.getPose(), FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT);

                        if (mFollowingPathChain) updateCallbacks();

                        mDrivePowers= mDriveVectorScaler.getDrivePowers(getCorrectiveVector(), getHeadingVector(), getDriveVector(), mPoseUpdater.getPose().getHeading());

                        for(int i_motor = 0; i_motor < mMotors.size(); i_motor ++) {
                            if (Math.abs(mMotors.get(i_motor).power() - mDrivePowers[i_motor]) > FollowerConstants.motorCachingThreshold) {
                                double mVoltageNormalized = getVoltageNormalized();

                                if (useVoltageCompensationInAuto) {
                                    mLogger.trace("Power " + mDrivePowers[i_motor] * mVoltageNormalized + " to motor " + mMotors.get(i_motor).name());
                                    mMotors.get(i_motor).power(mDrivePowers[i_motor] * mVoltageNormalized);
                                } else {
                                    mLogger.trace("Power " + mDrivePowers[i_motor] + " to motor " + mMotors.get(i_motor).name());
                                    mMotors.get(i_motor).power(mDrivePowers[i_motor]);
                                }
                            }
                            mMotors.get(i_motor).zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        }
                    }

                    // try to fix the robot stop near the end issue
                    // if robot is almost reach the end and velocity is close to zero
                    // then, break the following if other criteria meet
                    if (mPoseUpdater.getVelocity().getMagnitude() < 1.0 && mCurrentPath.getClosestPointTValue() > 0.8
                            && mZeroVelocityDetectedTimer == null && mIsBusy) {
                        mZeroVelocityDetectedTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                        mLogger.warning("!!!! Robot stuck !!!!");

                        debugLog();
                    }

                    if (mCurrentPath.isAtParametricEnd() ||
                            (mZeroVelocityDetectedTimer != null && mZeroVelocityDetectedTimer.milliseconds() > 500.0)) {
                        if (mFollowingPathChain && mChainIndex < mCurrentPathChain.size() - 1) {

                            mLogger.debug("mChainIndex: " + mChainIndex + " | Pose: " + getPose());
                            
                            // Not at last path, keep going
                            breakFollowing();
                            mPathStartTimes[mChainIndex]    = System.currentTimeMillis();
                            mIsBusy                         = true;
                            mFollowingPathChain             = true;
                            mChainIndex++;
                            mCurrentPath                    = mCurrentPathChain.getPath(mChainIndex);
                            mClosestPose                    = mCurrentPath.getClosestPoint(mPoseUpdater.getPose(), FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT);
                        } else {
                            // At last path, run some end detection stuff
                            // set mIsBusy to false if at end
                            if (!mReachedParametricPathEnd) {
                                mReachedParametricPathEnd = true;
                                mReachedParametricPathEndTime = System.currentTimeMillis();
                            }

                            if ((System.currentTimeMillis() - mReachedParametricPathEndTime > mCurrentPath.getPathEndTimeoutConstraint()) ||
                                    (mPoseUpdater.getVelocity().getMagnitude() < mCurrentPath.getPathEndVelocityConstraint()
                                            && MathFunctions.distance(mPoseUpdater.getPose(), mClosestPose) < mCurrentPath.getPathEndTranslationalConstraint() &&
                                            MathFunctions.getSmallestAngleDifference(mPoseUpdater.getPose().getHeading(), mCurrentPath.getClosestPointHeadingGoal()) < mCurrentPath.getPathEndHeadingConstraint())) {
                                if (mHoldPositionAtEnd) {
                                    mHoldPositionAtEnd = false;
                                    holdPoint(new BezierPoint(mCurrentPath.getLastControlPoint()), mCurrentPath.getHeadingGoal(1));
                                } else {
                                    if (mIsBusy) {
                                        mLogger.debug("isAtParametricEnd:" + mCurrentPath.isAtParametricEnd()
                                                + " | mIsBusy: " + mIsBusy
                                                + " | mClosestPose:" + mClosestPose
                                                + " | Pose: " + getPose()
                                                + " | t-value: " + String.format("%3.5f", mCurrentPath.getClosestPointTValue())
                                                + " | velocity: " + String.format("%3.2f", mPoseUpdater.getVelocity().getMagnitude())
                                                + " | distance: " + String.format("%3.2f", MathFunctions.distance(mPoseUpdater.getPose(), mClosestPose))
                                                + " | heading (degree): " + String.format("%3.2f", Math.toDegrees(MathFunctions.getSmallestAngleDifference(mPoseUpdater.getPose().getHeading(), mCurrentPath.getClosestPointHeadingGoal())))
                                        );
                                    }

                                    breakFollowing();
                                }
                            }
                        }
                    }
                    mLogger.debug("IsBusy:" + mIsBusy);
                }
            }
        } else {
            mVelocities.add(mPoseUpdater.getVelocity());
            mVelocities.remove(mVelocities.get(mVelocities.size() - 1));

            calculateAveragedVelocityAndAcceleration();

            mDrivePowers= mDriveVectorScaler.getDrivePowers(getCentripetalForceCorrection(), mTeleopHeadingVector, mTeleopDriveVector, mPoseUpdater.getPose().getHeading());

            for(int i_motor = 0; i_motor < mMotors.size(); i_motor ++) {
                if (Math.abs(mMotors.get(i_motor).power() - mDrivePowers[i_motor]) > FollowerConstants.motorCachingThreshold) {
                    double mVoltageNormalized = getVoltageNormalized();

                    if (useVoltageCompensationInAuto) {
                        mLogger.trace("Power " + mDrivePowers[i_motor] * mVoltageNormalized + " to motor " + mMotors.get(i_motor).name());
                        mMotors.get(i_motor).power(mDrivePowers[i_motor] * mVoltageNormalized);
                    } else {
                        mLogger.trace("Power " + mDrivePowers[i_motor] + " to motor " + mMotors.get(i_motor).name());
                        mMotors.get(i_motor).power(mDrivePowers[i_motor]);
                    }
                }
                mMotors.get(i_motor).zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        mLogger.debug(mName + " stop");
    }

    /**
     * This sets the teleop drive vectors. This defaults to robot centric.
     *
     * @param forwardDrive determines the forward drive vector for the robot in teleop. In field centric
     *                     movement, this is the x-axis.
     * @param lateralDrive determines the lateral drive vector for the robot in teleop. In field centric
     *                     movement, this is the y-axis.
     * @param heading determines the heading vector for the robot in teleop.
     */
    @Override
    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading) {
        setTeleOpMovementVectors(forwardDrive, lateralDrive, heading, true);
    }

    /**
     * This sets the teleop drive vectors.
     *
     * @param forwardDrive determines the forward drive vector for the robot in teleop. In field centric
     *                     movement, this is the x-axis.
     * @param lateralDrive determines the lateral drive vector for the robot in teleop. In field centric
     *                     movement, this is the y-axis.
     * @param heading determines the heading vector for the robot in teleop.
     * @param robotCentric sets if the movement will be field or robot centric
     */
    @Override
    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading, boolean robotCentric) {
        mTeleopDriveValues[0] = MathFunctions.clamp(forwardDrive, -1, 1);
        mTeleopDriveValues[1] = MathFunctions.clamp(lateralDrive, -1, 1);
        mTeleopDriveValues[2] = MathFunctions.clamp(heading, -1, 1);
        mTeleopDriveVector.setOrthogonalComponents(mTeleopDriveValues[0], mTeleopDriveValues[1]);
        mTeleopDriveVector.setMagnitude(MathFunctions.clamp(mTeleopDriveVector.getMagnitude(), 0, 1));

        if (robotCentric) {
            mTeleopDriveVector.rotateVector(getPose().getHeading());
        }

        mTeleopHeadingVector.setComponents(mTeleopDriveValues[2], getPose().getHeading());
    }

    /**
     * This calculates an averaged approximate velocity and acceleration. This is used for a
     * real-time correction of centripetal force, which is used in teleop.
     */
    @Override
    public void calculateAveragedVelocityAndAcceleration() {
        mAverageVelocity = new Vector();
        mAveragePreviousVelocity = new Vector();

        for (int i = 0; i < mVelocities.size() / 2; i++) {
            mAverageVelocity = MathFunctions.addVectors(mAverageVelocity, mVelocities.get(i));
        }
        mAverageVelocity = MathFunctions.scalarMultiplyVector(mAverageVelocity, 1.0 / ((double) mVelocities.size() / 2));

        for (int i = mVelocities.size() / 2; i < mVelocities.size(); i++) {
            mAveragePreviousVelocity = MathFunctions.addVectors(mAveragePreviousVelocity, mVelocities.get(i));
        }
        mAveragePreviousVelocity = MathFunctions.scalarMultiplyVector(mAveragePreviousVelocity, 1.0 / ((double) mVelocities.size() / 2));

        mAccelerations.add(MathFunctions.subtractVectors(mAverageVelocity, mAveragePreviousVelocity));
        mAccelerations.remove(mAccelerations.size() - 1);

        mAverageAcceleration = new Vector();

        for (int i = 0; i < mAccelerations.size(); i++) {
            mAverageAcceleration = MathFunctions.addVectors(mAverageAcceleration, mAccelerations.get(i));
        }
        mAverageAcceleration = MathFunctions.scalarMultiplyVector(mAverageAcceleration, 1.0 / mAccelerations.size());
    }

    /**
     * This checks if any PathCallbacks should be run right now, and runs them if applicable.
     */
    @Override
    public void updateCallbacks() {
        for (PathCallback callback : mCurrentPathChain.getCallbacks()) {
            if (!callback.hasBeenRun()) {
                if (callback.getType() == PathCallback.PARAMETRIC) {
                    // parametric call back
                    if (mChainIndex == callback.getIndex() && (getCurrentTValue() >= callback.getStartCondition() || MathFunctions.roughlyEquals(getCurrentTValue(), callback.getStartCondition()))) {
                        callback.run();
                    }
                } else {
                    // time based call back
                    if (mChainIndex >= callback.getIndex() && System.currentTimeMillis() - mPathStartTimes[callback.getIndex()] > callback.getStartCondition()) {
                        callback.run();
                    }

                }
            }
        }
    }

    /**
     * This resets the PIDFs and stops following the current Path.
     */
    @Override
    public void breakFollowing() {
        mTeleopDrive                            = false;
        setMotorsToFloat();
        mHoldingPosition                        = false;
        mIsBusy                                 = false;
        mReachedParametricPathEnd               = false;
        mSecondaryDrivePIDF.reset();
        mDrivePIDF.reset();
        mSecondaryHeadingPIDF.reset();
        mHeadingPIDF.reset();
        mSecondaryTranslationalPIDF.reset();
        mSecondaryTranslationalIntegral.reset();
        mSecondaryTranslationalIntegralVector   = new Vector();
        mPreviousSecondaryTranslationalIntegral = 0;
        mTranslationalPIDF.reset();
        mTranslationalIntegral.reset();
        mTranslationalIntegralVector            = new Vector();
        mPreviousTranslationalIntegral          = 0;
        mDriveVector                            = new Vector();
        mHeadingVector                          = new Vector();
        mTranslationalVector                    = new Vector();
        mCentripetalVector                      = new Vector();
        mCorrectiveVector                       = new Vector();
        mDriveError                             = 0;
        mHeadingError                           = 0;
        mRawDriveError                          = 0;
        mPreviousRawDriveError                  = 0;
        mDriveErrors = new double[2];
        Arrays.fill(mDriveErrors, 0);
        mDriveKalmanFilter.reset();

        for (int i = 0; i < FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER; i++) {
            mVelocities.add(new Vector());
        }
        for (int i = 0; i < FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER / 2; i++) {
            mAccelerations.add(new Vector());
        }
        calculateAveragedVelocityAndAcceleration();
        mTeleopDriveValues                      = new double[3];
        mTeleopDriveVector                      = new Vector();
        mTeleopHeadingVector                    = new Vector();

        for (int i = 0; i < mMotors.size(); i++) {
            mMotors.get(i).power(0);
        }

        mZeroVelocityDetectedTimer = null;
    }

    /**
     * This returns if the Follower is currently following a Path or a PathChain.
     *
     * @return returns if the Follower is busy.
     */
    @Override
    public boolean  isBusy() {
        return mIsBusy;
    }

    /**
     * This returns a Vector in the direction the robot must go to move along the path. This Vector
     * takes into account the projected position of the robot to calculate how much power is needed.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the drive vector.
     */
    @Override
    public Vector getDriveVector() {
        if (!mUseDrive) return new Vector();
        if (mFollowingPathChain && mChainIndex < mCurrentPathChain.size() - 1) {
            return new Vector(mDriveVectorScaler.getMaxPowerScaling(), mCurrentPath.getClosestPointTangentVector().getTheta());
        }

        mDriveError= getDriveVelocityError();

        if (Math.abs(mDriveError) < drivePIDFSwitch && useSecondaryDrivePID) {
            // Log.d("Follower_logger_secondary::", "In secondary drive PIDF");
            mSecondaryDrivePIDF.updateError(mDriveError);
            mDriveVector = new Vector(MathFunctions.clamp(mSecondaryDrivePIDF.runPIDF() + secondaryDrivePIDFFeedForward * MathFunctions.getSign(mDriveError), -mDriveVectorScaler.getMaxPowerScaling(), mDriveVectorScaler.getMaxPowerScaling()), mCurrentPath.getClosestPointTangentVector().getTheta());
            return MathFunctions.copyVector(mDriveVector);
        }

        mDrivePIDF.updateError(mDriveError);
        mDriveVector = new Vector(MathFunctions.clamp(mDrivePIDF.runPIDF() + drivePIDFFeedForward * MathFunctions.getSign(mDriveError), -mDriveVectorScaler.getMaxPowerScaling(), mDriveVectorScaler.getMaxPowerScaling()), mCurrentPath.getClosestPointTangentVector().getTheta());
        return MathFunctions.copyVector(mDriveVector);
    }

    /**
     * This returns the velocity the robot needs to be at to make it to the end of the Path
     * at some specified deceleration (well technically just some negative acceleration).
     *
     * @return returns the projected velocity.
     */
    @Override
    public double getDriveVelocityError() {
        double distanceToGoal;
        if (!mCurrentPath.isAtParametricEnd()) {
            distanceToGoal = mCurrentPath.length() * (1 - mCurrentPath.getClosestPointTValue());
        } else {
            Vector offset = new Vector();
            offset.setOrthogonalComponents(getPose().getX() - mCurrentPath.getLastControlPoint().getX(), getPose().getY() - mCurrentPath.getLastControlPoint().getY());
            distanceToGoal = MathFunctions.dotProduct(mCurrentPath.getEndTangent(), offset);
        }

        Vector distanceToGoalVector = MathFunctions.scalarMultiplyVector(MathFunctions.normalizeVector(mCurrentPath.getClosestPointTangentVector()), distanceToGoal);
        Vector velocity = new Vector(MathFunctions.dotProduct(getVelocity(), MathFunctions.normalizeVector(mCurrentPath.getClosestPointTangentVector())), mCurrentPath.getClosestPointTangentVector().getTheta());

        Vector forwardHeadingVector = new Vector(1.0, mPoseUpdater.getPose().getHeading());

        double forwardVelocity = MathFunctions.dotProduct(forwardHeadingVector, velocity);
        double forwardDistanceToGoal = MathFunctions.dotProduct(forwardHeadingVector, distanceToGoalVector);
        double forwardVelocityGoal = MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(-2 * mCurrentPath.getZeroPowerAccelerationMultiplier() * forwardZeroPowerAcceleration * (forwardDistanceToGoal <= 0 ? 1 : -1) * forwardDistanceToGoal));
        double forwardVelocityZeroPowerDecay = forwardVelocity - MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(forwardVelocity, 2) + 2 * forwardZeroPowerAcceleration * Math.abs(forwardDistanceToGoal)));

        Vector lateralHeadingVector = new Vector(1.0, mPoseUpdater.getPose().getHeading() - Math.PI / 2);
        double lateralVelocity = MathFunctions.dotProduct(lateralHeadingVector, velocity);
        double lateralDistanceToGoal = MathFunctions.dotProduct(lateralHeadingVector, distanceToGoalVector);

        double lateralVelocityGoal = MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(-2 * mCurrentPath.getZeroPowerAccelerationMultiplier() * lateralZeroPowerAcceleration * (lateralDistanceToGoal <= 0 ? 1 : -1) * lateralDistanceToGoal));
        double lateralVelocityZeroPowerDecay = lateralVelocity - MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(lateralVelocity, 2) + 2 * lateralZeroPowerAcceleration * Math.abs(lateralDistanceToGoal)));

        Vector forwardVelocityError = new Vector(forwardVelocityGoal - forwardVelocityZeroPowerDecay - forwardVelocity, forwardHeadingVector.getTheta());
        Vector lateralVelocityError = new Vector(lateralVelocityGoal - lateralVelocityZeroPowerDecay - lateralVelocity, lateralHeadingVector.getTheta());
        Vector velocityErrorVector = MathFunctions.addVectors(forwardVelocityError, lateralVelocityError);

        mPreviousRawDriveError = mRawDriveError;
        mRawDriveError = velocityErrorVector.getMagnitude() * MathFunctions.getSign(MathFunctions.dotProduct(velocityErrorVector, mCurrentPath.getClosestPointTangentVector()));

        double projection = 2 * mDriveErrors[1] - mDriveErrors[0];

        mDriveKalmanFilter.update(mRawDriveError - mPreviousRawDriveError, projection);

        for (int i = 0; i < mDriveErrors.length - 1; i++) {
            mDriveErrors[i] = mDriveErrors[i + 1];
        }
        mDriveErrors[1] = mDriveKalmanFilter.getState();

        return mDriveKalmanFilter.getState();
    }
    /**
     * This returns a Vector in the direction of the robot that contains the heading correction
     * as its magnitude. Positive heading correction turns the robot counter-clockwise, and negative
     * heading correction values turn the robot clockwise. So basically, Pedro Pathing uses a right-
     * handed coordinate system.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the heading vector.
     */
    @Override
    public Vector getHeadingVector() {
        if (!mUseHeading) return new Vector();
        mHeadingError= MathFunctions.getTurnDirection(mPoseUpdater.getPose().getHeading(), mCurrentPath.getClosestPointHeadingGoal()) * MathFunctions.getSmallestAngleDifference(mPoseUpdater.getPose().getHeading(), mCurrentPath.getClosestPointHeadingGoal());
        if (Math.abs(mHeadingError) < headingPIDFSwitch && useSecondaryHeadingPID) {
//            if(logDebug) {
//                Log.d("Follower_logger", "using secondary heading PIDF controller, error: "
//                        + String.format("%3.3f", Math.toDegrees(headingError)));
//
//            }
            mSecondaryHeadingPIDF.updateError(mHeadingError);
            mHeadingVector = new Vector(MathFunctions.clamp(mSecondaryHeadingPIDF.runPIDF() + secondaryHeadingPIDFFeedForward * MathFunctions.getTurnDirection(mPoseUpdater.getPose().getHeading(), mCurrentPath.getClosestPointHeadingGoal()), -mDriveVectorScaler.getMaxPowerScaling(), mDriveVectorScaler.getMaxPowerScaling()), mPoseUpdater.getPose().getHeading());
            return MathFunctions.copyVector(mHeadingVector);
        }
        mHeadingPIDF.updateError(mHeadingError);
        mHeadingVector = new Vector(MathFunctions.clamp(mHeadingPIDF.runPIDF() + headingPIDFFeedForward * MathFunctions.getTurnDirection(mPoseUpdater.getPose().getHeading(), mCurrentPath.getClosestPointHeadingGoal()), -mDriveVectorScaler.getMaxPowerScaling(), mDriveVectorScaler.getMaxPowerScaling()), mPoseUpdater.getPose().getHeading());
        return MathFunctions.copyVector(mHeadingVector);
    }

    /**
     * This returns a combined Vector in the direction the robot must go to correct both translational
     * error as well as centripetal force.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the corrective vector.
     */
    @Override
    public Vector getCorrectiveVector() {
        Vector centripetal = getCentripetalForceCorrection();
        Vector translational = getTranslationalCorrection();
        Vector corrective = MathFunctions.addVectors(centripetal, translational);

        if (corrective.getMagnitude() > mDriveVectorScaler.getMaxPowerScaling()) {
            return MathFunctions.addVectors(centripetal, MathFunctions.scalarMultiplyVector(translational, mDriveVectorScaler.findNormalizingScaling(centripetal, translational)));
        }

        mCorrectiveVector = MathFunctions.copyVector(corrective);

        return corrective;
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only translational
     * error.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the translational correction vector.
     */
    @Override
    public Vector getTranslationalCorrection() {
        if (!mUseTranslational) return new Vector();
        Vector mTranslationalVector = new Vector();
        double x = mClosestPose.getX() - mPoseUpdater.getPose().getX();
        double y = mClosestPose.getY() - mPoseUpdater.getPose().getY();
        mTranslationalVector.setOrthogonalComponents(x, y);

        if (!(mCurrentPath.isAtParametricEnd() || mCurrentPath.isAtParametricStart())) {
            mTranslationalVector = MathFunctions.subtractVectors(mTranslationalVector, new Vector(MathFunctions.dotProduct(mTranslationalVector, MathFunctions.normalizeVector(mCurrentPath.getClosestPointTangentVector())), mCurrentPath.getClosestPointTangentVector().getTheta()));

            mSecondaryTranslationalIntegralVector = MathFunctions.subtractVectors(mSecondaryTranslationalIntegralVector, new Vector(MathFunctions.dotProduct(mSecondaryTranslationalIntegralVector, MathFunctions.normalizeVector(mCurrentPath.getClosestPointTangentVector())), mCurrentPath.getClosestPointTangentVector().getTheta()));
            mTranslationalIntegralVector = MathFunctions.subtractVectors(mTranslationalIntegralVector, new Vector(MathFunctions.dotProduct(mTranslationalIntegralVector, MathFunctions.normalizeVector(mCurrentPath.getClosestPointTangentVector())), mCurrentPath.getClosestPointTangentVector().getTheta()));
        }

        if (MathFunctions.distance(mPoseUpdater.getPose(), mClosestPose) < translationalPIDFSwitch && useSecondaryTranslationalPID) {
            mSecondaryTranslationalIntegral.updateError(mTranslationalVector.getMagnitude());
            mSecondaryTranslationalIntegralVector = MathFunctions.addVectors(mSecondaryTranslationalIntegralVector, new Vector(mSecondaryTranslationalIntegral.runPIDF() - mPreviousSecondaryTranslationalIntegral, mTranslationalVector.getTheta()));
            mPreviousSecondaryTranslationalIntegral = mSecondaryTranslationalIntegral.runPIDF();

            mSecondaryTranslationalPIDF.updateError(mTranslationalVector.getMagnitude());
            mTranslationalVector.setMagnitude(mSecondaryTranslationalPIDF.runPIDF() + secondaryTranslationalPIDFFeedForward);
            mTranslationalVector = MathFunctions.addVectors(mTranslationalVector, mSecondaryTranslationalIntegralVector);
        } else {
            mTranslationalIntegral.updateError(mTranslationalVector.getMagnitude());
            mTranslationalIntegralVector = MathFunctions.addVectors(mTranslationalIntegralVector, new Vector(mTranslationalIntegral.runPIDF() - mPreviousTranslationalIntegral, mTranslationalVector.getTheta()));
            mPreviousTranslationalIntegral = mTranslationalIntegral.runPIDF();

            mTranslationalPIDF.updateError(mTranslationalVector.getMagnitude());
            mTranslationalVector.setMagnitude(mTranslationalPIDF.runPIDF() + translationalPIDFFeedForward);
            mTranslationalVector = MathFunctions.addVectors(mTranslationalVector, mTranslationalIntegralVector);
        }

        mTranslationalVector.setMagnitude(MathFunctions.clamp(mTranslationalVector.getMagnitude(), 0, mDriveVectorScaler.getMaxPowerScaling()));

        this.mTranslationalVector = MathFunctions.copyVector(mTranslationalVector);

        return mTranslationalVector;
    }

    /**
     * This returns the raw translational error, or how far off the closest point the robot is.
     *
     * @return This returns the raw translational error as a Vector.
     */
    @Override
    public Vector getTranslationalError() {
        Vector error = new Vector();
        double x = mClosestPose.getX() - mPoseUpdater.getPose().getX();
        double y = mClosestPose.getY() - mPoseUpdater.getPose().getY();
        error.setOrthogonalComponents(x, y);
        return error;
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only centripetal
     * force.
     * <p>
     * Note: This vector is clamped to be between [0, 1] in magnitude.
     *
     * @return returns the centripetal force correction vector.
     */
    @Override
    public Vector getCentripetalForceCorrection() {
        if (!mUseCentripetal) return new Vector();
        double curvature;
        if (!mTeleopDrive) {
            curvature = mCurrentPath.getClosestPointCurvature();
        } else {
            double yPrime = mAverageVelocity.getYComponent() / mAverageVelocity.getXComponent();
            double yDoublePrime = mAverageAcceleration.getYComponent() / mAverageVelocity.getXComponent();
            curvature = (yDoublePrime) / (Math.pow(Math.sqrt(1 + Math.pow(yPrime, 2)), 3));
        }
        if (Double.isNaN(curvature)) return new Vector();
        mCentripetalVector = new Vector(MathFunctions.clamp(centripetalScaling * FollowerConstants.mass * Math.pow(MathFunctions.dotProduct(mPoseUpdater.getVelocity(), MathFunctions.normalizeVector(mCurrentPath.getClosestPointTangentVector())), 2) * curvature, -mDriveVectorScaler.getMaxPowerScaling(), mDriveVectorScaler.getMaxPowerScaling()), mCurrentPath.getClosestPointTangentVector().getTheta() + Math.PI / 2 * MathFunctions.getSign(mCurrentPath.getClosestPointNormalVector().getTheta()));
        return mCentripetalVector;
    }

    /**
     * This returns the closest pose to the robot on the Path the Follower is currently following.
     * This closest pose is calculated through a binary search method with some specified number of
     * steps to search. By default, 10 steps are used, which should be more than enough.
     *
     * @return returns the closest pose.
     */
    @Override
    public Pose getClosestPose() {
        return mClosestPose;
    }

    /**
     * This returns whether the follower is at the parametric end of its current Path.
     * The parametric end is determined by if the closest Point t-value is greater than some specified
     * end t-value.
     * If running a PathChain, this returns true only if at parametric end of last Path in the PathChain.
     *
     * @return returns whether the Follower is at the parametric end of its Path.
     */
    @Override
    public boolean atParametricEnd() {
        if (mFollowingPathChain) {
            if (mChainIndex == mCurrentPathChain.size() - 1) return mCurrentPath.isAtParametricEnd();
            return false;
        }
        return mCurrentPath.isAtParametricEnd();
    }

    /**
     * This returns the t value of the closest point on the current Path to the robot
     * In the absence of a current Path, it returns 1.0.
     *
     * @return returns the current t value.
     */
    @Override
    public double getCurrentTValue() {
        if (mIsBusy) return mCurrentPath.getClosestPointTValue();
        return 1.0;
    }

    /**
     * This returns the current path number. For following Paths, this will return 0. For PathChains,
     * this will return the current path number. For holding Points, this will also return 0.
     *
     * @return returns the current path number.
     */
    @Override
    public double getCurrentPathNumber() {
        if (!mFollowingPathChain) return 0;
        return mChainIndex;
    }

    /**
     * This returns a new PathBuilder object for easily building PathChains.
     *
     * @return returns a new PathBuilder object.
     */
    @Override
    public PathBuilder pathBuilder() {
        return new PathBuilder();
    }

    /**
     * This writes out information about the various motion Vectors to the Telemetry specified.
     *
     */
    public void telemetryDebug() {

        String log = "follower busy : " + isBusy() +
                "\nfollower busy : " + mHeadingError +
                "\nheading vector magnitude : " + mHeadingVector.getMagnitude() +
                "\ncorrective vector magnitude : " + mCorrectiveVector.getMagnitude() +
                "\ncorrective vector heading : " + mCorrectiveVector.getTheta() +
                "\ntranslational error magnitude : " + getTranslationalError().getMagnitude() +
                "\ntranslational error direction : " + getTranslationalError().getTheta() +
                "\ntranslational vector magnitude : " + mTranslationalVector.getMagnitude() +
                "\ntranslational vector heading : " + mTranslationalVector.getTheta() +
                "\ncentripetal vector magnitude : " + mCentripetalVector.getMagnitude() +
                "\ncentripetal vector heading : " + mCentripetalVector.getTheta() +
                "\ndrive error : " + mDriveError +
                "\ndrive vector magnitude : " + mDriveVector.getMagnitude() +
                "\ndrive vector heading : " + mDriveVector.getTheta() +
                "\nx : " + getPose().getX() +
                "\ny : " + getPose().getY() +
                "\nheading : " + getPose().getHeading() +
                "\ntotal heading : " + mPoseUpdater.getTotalHeading() +
                "\nclosest x : " + mClosestPose.getX() +
                "\nclosest y : " + mClosestPose.getY() +
                "\nclosest heading : " + mClosestPose.getHeading() +
                "\nvelocity magnitude : " + getVelocity().getMagnitude() +
                "\nvelocity heading : " + getVelocity().getTheta();

        mLogger.debug(log);
        if (mDrawOnDashboard) { Drawing.drawDebug(this); }

    }

    /**
     * This returns the total number of radians the robot has turned.
     *
     * @return the total heading.
     */
    @Override
    public double getTotalHeading() {
        return mPoseUpdater.getTotalHeading();
    }

    /**
     * This returns the current Path the Follower is following. This can be null.
     *
     * @return returns the current Path.
     */
    @Override
    public Path getCurrentPath() {
        return mCurrentPath;
    }

    /**
     * This returns the pose tracker for the robot to draw on the Dashboard.
     *
     * @return returns the pose tracker
     */
    @Override
    public DashboardPoseTracker getDashboardPoseTracker() {
        return mDashboardPoseTracker;
    }

    /**
     * This resets the IMU, if applicable.
     */
    private void resetIMU() throws InterruptedException {
        mPoseUpdater.resetIMU();
    }

    private void debugLog() {
        mLogger.debug("isAtParametricEnd:" + mCurrentPath.isAtParametricEnd()
                + " | mIsBusy: " + mIsBusy
                + " | mClosestPose:" + mClosestPose
                + " | Pose: " + getPose()
                + " | t-value: " + String.format("%3.5f",mCurrentPath.getClosestPointTValue())
                + " | zeroVelocityTimer: " +  String.format("%3.2f",(mZeroVelocityDetectedTimer==null?0.0: mZeroVelocityDetectedTimer.milliseconds()))
                + " | velocity: " + String.format("%3.2f",mPoseUpdater.getVelocity().getMagnitude())
                + " | distance: " +  String.format("%3.2f",MathFunctions.distance(mPoseUpdater.getPose(), mClosestPose))
                + " | heading (degree): " +  String.format("%3.2f",Math.toDegrees(MathFunctions.getSmallestAngleDifference(mPoseUpdater.getPose().getHeading(), mCurrentPath.getClosestPointHeadingGoal())))
        );
    }

    //Thanks to team 21229 Quality Control for creating this algorithm to detect if the robot is stuck.
    /**
     * @return true if the robot is stuck and false otherwise
     */
    @Override
    public boolean isRobotStuck() {
        return mZeroVelocityDetectedTimer != null;
    }

    /**
     * Draws everything in the debug() method on the dashboard
     */

    @Override
    public void drawOnDashBoard() {
        if (mDrawOnDashboard) { Drawing.drawDebug(this); }
    }

    @Override
    public boolean isLocalizationNAN() {
        return mPoseUpdater.getLocalizer().isNAN();
    }

    /**
     * @return The last mCached mVoltage measurement.
     */
    @Override
    public double getVoltage() {
        if (mVoltageTimer.seconds() > cacheInvalidateSeconds && cacheInvalidateSeconds >= 0) {
            mCached = false;
        }

        if (!mCached)
            refreshVoltage();

        return mVoltage;
    }

    /**
     * @return A scalar that normalizes power outputs to the nominal mVoltage from the current mVoltage.
     */
    @Override
    public double getVoltageNormalized() {
        return Math.min(nominalVoltage / getVoltage(), 1);
    }

    /**
     * Overrides the mVoltage cooldown.
     */
    @Override
    public void refreshVoltage() {
        mCached = true;
        mVoltage = mVoltageSensor.getVoltage();
        mVoltageTimer.reset();
    }

    /** Turns a certain amount of degrees left
     * @param radians the amount of radians to turn
     * @param isLeft true if turning left, false if turning right
     */
    @Override
    public void turn(double radians, boolean isLeft) {
        Pose temp = new Pose(getPose().getX(), getPose().getY(), getPose().getHeading() + (isLeft ? radians : -radians));
        holdPoint(temp);
    }

    /** Turns to a specific heading
     * @param radians the heading in radians to turn to
     */
    @Override
    public void turnTo(double radians) {
        holdPoint(new Pose(getPose().getX(), getPose().getY(), Math.toRadians(radians)));
    }

    /** Turns to a specific heading in degrees
     * @param degrees the heading in degrees to turn to
     */
    @Override
    public void turnToDegrees(double degrees) {
        turnTo(Math.toRadians(degrees));
    }

    /** Turns a certain amount of degrees left
     * @param degrees the amount of degrees to turn
     * @param isLeft true if turning left, false if turning right
     */
    @Override
    public void                         turnDegrees(double degrees, boolean isLeft) {
        turn(Math.toRadians(degrees), isLeft);
    }

    
    public void                         drawOnDashboard(boolean value) {
        if (mConfigurationValid) { mDrawOnDashboard = value; }
    }
    public void                         useTranslational(boolean value) {
        if (mConfigurationValid) { mUseTranslational = value; }
    }
    public void                         useCentripetal(boolean value) {
        if (mConfigurationValid) { mUseCentripetal = value; }
    }
    public void                         useHeading(boolean value) {
        if (mConfigurationValid) { mUseHeading = value; }
    }
    public void                         useDrive(boolean value) {
        if (mConfigurationValid) { mUseDrive = value; }
    }
    
    /**
     * Determines if the actuator subsystem is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /**
     * Reads and applies the mecanum drive configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    public void                         read(JSONObject reader) {

        mConfigurationValid     = true;

        mLeftFront              = null;
        mRightFront             = null;
        mLeftBack               = null;
        mRightBack              = null;
        mLocalizer              = null;

        try {

            if(reader.has(sShortNameKey)) {
                mShortName = reader.getString(sShortNameKey);
            }
            if(mShortName.isEmpty()) { mShortName = mName; }

            if (reader.has(sReferenceKey)) {
                String reference = reader.getString(sReferenceKey);
                if(reference.equals(sFieldCentricKey)) { mDrivingMode = Mode.FIELD_CENTRIC; }
                if(reference.equals(sRobotCentricKey)) { mDrivingMode = Mode.ROBOT_CENTRIC; }
            }

            if(reader.has(sMotorsKey)) {
                Map<String,MotorComponent> motors = mHardware.motors();
                JSONObject wheels  = reader.getJSONObject(sMotorsKey);

                if(wheels.has(sFrontLeftKey)) {
                    mLeftFrontHwName = wheels.getString(sFrontLeftKey);
                    if (motors.containsKey(mLeftFrontHwName)) {
                        mLeftFront = motors.get(mLeftFrontHwName);
                        if(mLeftFront != null) {
                            mLeftFront.mode(DcMotor.RunMode.RUN_USING_ENCODER);
                            mLeftFront.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        }
                    }
                }
                if(wheels.has(sBackLeftKey)) {
                    mLeftBackHwName = wheels.getString(sBackLeftKey);
                    if (motors.containsKey(mLeftBackHwName)) {
                        mLeftBack = motors.get(mLeftBackHwName);
                        if(mLeftBack != null) {
                            mLeftBack.mode(DcMotor.RunMode.RUN_USING_ENCODER);
                            mLeftBack.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        }
                    }
                }
                if(wheels.has(sFrontRightKey)) {
                    mRightFrontHwName = wheels.getString(sFrontRightKey);
                    if (motors.containsKey(mRightFrontHwName)) {
                        mRightFront = motors.get(mRightFrontHwName);
                        if(mRightFront != null ) {
                            mRightFront.mode(DcMotor.RunMode.RUN_USING_ENCODER);
                            mRightFront.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        }
                    }
                }
                if(wheels.has(sBackRightKey)) {
                    mRightBackHwName = wheels.getString(sBackRightKey);
                    if (motors.containsKey(mRightBackHwName)) {
                        mRightBack = motors.get(mRightBackHwName);
                        if(mRightBack != null) {
                            mRightBack.mode(DcMotor.RunMode.RUN_USING_ENCODER);
                            mRightBack.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        }
                    }
                }
            }

            if(reader.has(sLocalizerKey)) {
                Map<String,LocalizerComponent> localizers = mHardware.localizers();
                mLocalizerHwName = reader.getString(sLocalizerKey);
                if (localizers.containsKey(mLocalizerHwName)) {
                    mLocalizer = localizers.get(mLocalizerHwName);
                }
            }

            if(reader.has(sFollowerKey)) {

                JSONObject follower = reader.getJSONObject(sFollowerKey);

                FollowerConstants.motorCachingThreshold = 0.01;
                if (follower.has(sMotorCachingThresholdKey)) {
                    FollowerConstants.motorCachingThreshold = follower.getDouble(sMotorCachingThresholdKey);
                }

                FollowerConstants.xMovement = 81.34056;
                if (follower.has(sXMovementKey)) {
                    FollowerConstants.xMovement = follower.getDouble(sXMovementKey);
                }

                FollowerConstants.yMovement = 65.43028;
                if (follower.has(sYMovementKey)) {
                    FollowerConstants.yMovement = follower.getDouble(sYMovementKey);
                }

                FollowerConstants.maxPower = 1.0;
                if (follower.has(sMaxPowerKey)) {
                    FollowerConstants.maxPower = follower.getDouble(sMaxPowerKey);
                }

                double p = 0.1;
                double i = 0,f = 0,d = 0;
                if(follower.has(sTranslationPidfKey)) {
                    JSONObject pid = follower.getJSONObject(sTranslationPidfKey);
                    if(pid.has(sPidfPKey)) { p = pid.getDouble(sPidfPKey); }
                    if(pid.has(sPidfIKey)) { i = pid.getDouble(sPidfIKey); }
                    if(pid.has(sPidfDKey)) { d = pid.getDouble(sPidfDKey); }
                    if(pid.has(sPidfFKey)) { f = pid.getDouble(sPidfFKey); }
                }
                FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(p,i,d,f);

                p = i = d = f = 0;
                if(follower.has(sTranslationIntegralKey)) {
                    JSONObject pid = follower.getJSONObject(sTranslationIntegralKey);
                    if(pid.has(sPidfPKey)) { p = pid.getDouble(sPidfPKey); }
                    if(pid.has(sPidfIKey)) { i = pid.getDouble(sPidfIKey); }
                    if(pid.has(sPidfDKey)) { d = pid.getDouble(sPidfDKey); }
                    if(pid.has(sPidfFKey)) { f = pid.getDouble(sPidfFKey); }
                }
                FollowerConstants.translationalIntegral  = new CustomPIDFCoefficients(p,i,d,f);

                FollowerConstants.translationalPIDFFeedForward = 0.015;
                if(follower.has(sTranslationPidfFFKey)) {
                    FollowerConstants.translationalPIDFFeedForward = follower.getDouble(sTranslationPidfFFKey);
                }

                p = 1.0;
                i = d = f = 0;
                if(follower.has(sHeadingPidfKey)) {
                    JSONObject pid = follower.getJSONObject(sHeadingPidfKey);
                    if(pid.has(sPidfPKey)) { p = pid.getDouble(sPidfPKey); }
                    if(pid.has(sPidfIKey)) { i = pid.getDouble(sPidfIKey); }
                    if(pid.has(sPidfDKey)) { d = pid.getDouble(sPidfDKey); }
                    if(pid.has(sPidfFKey)) { f = pid.getDouble(sPidfFKey); }
                }
                FollowerConstants.headingPIDFCoefficients   = new CustomPIDFCoefficients(p,i,d,f);

                FollowerConstants.headingPIDFFeedForward  = 0.01;
                if(follower.has(sHeadingPidfFFKey)) {
                    FollowerConstants.headingPIDFFeedForward  = follower.getDouble(sHeadingPidfFFKey);
                }

                double t = 0.6;
                p = 0.025;
                d = 0.00001;
                f = i = 0;
                if(follower.has(sDrivePidfKey)) {
                    JSONObject pid = follower.getJSONObject(sDrivePidfKey);
                    if(pid.has(sPidfPKey)) { p = pid.getDouble(sPidfPKey); }
                    if(pid.has(sPidfIKey)) { i = pid.getDouble(sPidfIKey); }
                    if(pid.has(sPidfDKey)) { d = pid.getDouble(sPidfDKey); }
                    if(pid.has(sPidfFKey)) { f = pid.getDouble(sPidfFKey); }
                    if(pid.has(sPidfTKey)) { t = pid.getDouble(sPidfTKey); }
                }
                FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(p,i,d,t,f);

                FollowerConstants.drivePIDFFeedForward  = 0.01;
                if(follower.has(sDrivePidfFFKey)) {
                    FollowerConstants.drivePIDFFeedForward  = follower.getDouble(sDrivePidfFFKey);
                }

                double model = 6;
                double data = 1;
                if(follower.has(sDriveKalmanFilterKey)) {
                    JSONObject kalman = follower.getJSONObject(sDriveKalmanFilterKey);
                    if(kalman.has(sKalmanModelKey)) { model = kalman.getDouble(sKalmanModelKey); }
                    if(kalman.has(sKalmanDataKey)) { data = kalman.getDouble(sKalmanDataKey); }
                }
                FollowerConstants.driveKalmanFilterParameters = new KalmanFilterParameters(model, data);

                FollowerConstants.mass = 10.65942;
                if(follower.has(sMassKey)) {
                    FollowerConstants.mass  = follower.getDouble(sMassKey);
                }
                FollowerConstants.centripetalScaling = 0.0005;
                if(follower.has(sCentripetalScalingKey)) {
                    FollowerConstants.centripetalScaling  = follower.getDouble(sCentripetalScalingKey);
                }

                FollowerConstants.forwardZeroPowerAcceleration = -34.62719;
                if(follower.has(sForward0PowerAccKey)) {
                    FollowerConstants.forwardZeroPowerAcceleration  = follower.getDouble(sForward0PowerAccKey);
                }

                FollowerConstants.lateralZeroPowerAcceleration = -78.15554;
                if(follower.has(sStrafe0PowerAccKey)) {
                    FollowerConstants.lateralZeroPowerAcceleration  = follower.getDouble(sStrafe0PowerAccKey);
                }

                FollowerConstants.zeroPowerAccelerationMultiplier = 4;
                if(follower.has(s0PowerAccMultiplierKey)) {
                    FollowerConstants.zeroPowerAccelerationMultiplier  = follower.getDouble(s0PowerAccMultiplierKey);
                }

                FollowerConstants.pathEndVelocityConstraint = 0.1;
                if(follower.has(sPathEndVelocityConstKey)) {
                    FollowerConstants.pathEndVelocityConstraint  = follower.getDouble(sPathEndVelocityConstKey);
                }

                FollowerConstants.pathEndTranslationalConstraint = 0.1;
                if(follower.has(sPathEndTranslationConstKey)) {
                    FollowerConstants.pathEndTranslationalConstraint  = follower.getDouble(sPathEndTranslationConstKey);
                }

                FollowerConstants.pathEndHeadingConstraint = 0.007;
                if(follower.has(sPathEndHeadingConstKey)) {
                    FollowerConstants.pathEndTranslationalConstraint  = follower.getDouble(sPathEndHeadingConstKey);
                }

                FollowerConstants.pathEndTValueConstraint = 0.995;
                if(follower.has(sPathEndValueConstKey)) {
                    FollowerConstants.pathEndTValueConstraint  = follower.getDouble(sPathEndValueConstKey);
                }

                FollowerConstants.pathEndTimeoutConstraint = 500;
                if(follower.has(sPathEndTimeOutConstKey)) {
                    FollowerConstants.pathEndTimeoutConstraint  = follower.getDouble(sPathEndTimeOutConstKey);
                }

                FollowerConstants.APPROXIMATION_STEPS = 1000;
                if(follower.has(sApproximationSteps)) {
                    FollowerConstants.APPROXIMATION_STEPS  = follower.getInt(sApproximationSteps);
                }

                FollowerConstants.holdPointTranslationalScaling = 0.45;
                if(follower.has(sHoldPointTranslationScalingKey)) {
                    FollowerConstants.holdPointTranslationalScaling  = follower.getDouble(sHoldPointTranslationScalingKey);
                }

                FollowerConstants.holdPointHeadingScaling = 0.35;
                if(follower.has(sHoldPointHeadingScalingKey)) {
                    FollowerConstants.holdPointHeadingScaling  = follower.getDouble(sHoldPointHeadingScalingKey);
                }

                FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER = 8;
                if(follower.has(sAverageVelocitySampleKey)) {
                    FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER  = follower.getInt(sAverageVelocitySampleKey);
                }

                FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT = 10;
                if(follower.has(sBezierCurveSearchLimitKey)) {
                    FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT  = follower.getInt(sBezierCurveSearchLimitKey);
                }
                FollowerConstants.useSecondaryTranslationalPID = follower.has(sSecondTranslationPidfKey) || follower.has(sTranslationPidfSwitch) || follower.has(sSecondTranslationIntegralKey) || follower.has(sSecondTranslationPidfFFKey);
                FollowerConstants.useSecondaryHeadingPID = follower.has(sSecondHeadingPidfKey) || follower.has(sHeadingPidfSwitch) || follower.has(sSecondHeadingPidfFFKey);
                FollowerConstants.useSecondaryDrivePID = follower.has(sSecondDrivePidfKey) || follower.has(sDrivePidfSwitch) || follower.has(sSecondDrivePidfFFKey);

                p = 0.3;
                i = f = 0;
                d = 0.01;
                if(follower.has(sSecondTranslationPidfKey)) {
                    JSONObject pid = follower.getJSONObject(sSecondTranslationPidfKey);
                    if(pid.has(sPidfPKey)) { p = pid.getDouble(sPidfPKey); }
                    if(pid.has(sPidfIKey)) { i = pid.getDouble(sPidfIKey); }
                    if(pid.has(sPidfDKey)) { d = pid.getDouble(sPidfDKey); }
                    if(pid.has(sPidfFKey)) { f = pid.getDouble(sPidfFKey); }
                }
                FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(p,i,d,f);

                p = i = d = f = 0;
                if(follower.has(sSecondTranslationIntegralKey)) {
                    JSONObject pid = follower.getJSONObject(sSecondTranslationIntegralKey);
                    if(pid.has(sPidfPKey)) { p = pid.getDouble(sPidfPKey); }
                    if(pid.has(sPidfIKey)) { i = pid.getDouble(sPidfIKey); }
                    if(pid.has(sPidfDKey)) { d = pid.getDouble(sPidfDKey); }
                    if(pid.has(sPidfFKey)) { f = pid.getDouble(sPidfFKey); }
                }
                FollowerConstants.secondaryTranslationalIntegral = new CustomPIDFCoefficients(p,i,d,f);

                FollowerConstants.secondaryTranslationalPIDFFeedForward = 0.015;
                if(follower.has(sSecondTranslationPidfFFKey)) {
                    FollowerConstants.secondaryTranslationalPIDFFeedForward = follower.getDouble(sSecondTranslationPidfFFKey);
                }

                FollowerConstants.translationalPIDFSwitch = 3;
                if(follower.has(sTranslationPidfSwitch)) {
                    FollowerConstants.translationalPIDFSwitch = follower.getDouble(sTranslationPidfSwitch);
                }

                p = 5.0;
                i = f = 0;
                d = 0.08;
                if(follower.has(sSecondHeadingPidfKey)) {
                    JSONObject pid = follower.getJSONObject(sSecondHeadingPidfKey);
                    if(pid.has(sPidfPKey)) { p = pid.getDouble(sPidfPKey); }
                    if(pid.has(sPidfIKey)) { i = pid.getDouble(sPidfIKey); }
                    if(pid.has(sPidfDKey)) { d = pid.getDouble(sPidfDKey); }
                    if(pid.has(sPidfFKey)) { f = pid.getDouble(sPidfFKey); }
                }
                FollowerConstants.secondaryHeadingPIDFCoefficients   = new CustomPIDFCoefficients(p,i,d,f);

                FollowerConstants.secondaryHeadingPIDFFeedForward  = 0.01;
                if(follower.has(sSecondHeadingPidfFFKey)) {
                    FollowerConstants.secondaryHeadingPIDFFeedForward  = follower.getDouble(sSecondHeadingPidfFFKey);
                }
                
                FollowerConstants.headingPIDFSwitch = Math.PI / 20;
                if(follower.has(sHeadingPidfSwitch)) {
                    FollowerConstants.headingPIDFSwitch = follower.getDouble(sHeadingPidfSwitch);
                }

                t = 0.6;
                p = 0.02;
                d = 0.000005;
                f = i = 0;
                if(follower.has(sSecondDrivePidfKey)) {
                    JSONObject pid = follower.getJSONObject(sSecondDrivePidfKey);
                    if(pid.has(sPidfPKey)) { p = pid.getDouble(sPidfPKey); }
                    if(pid.has(sPidfIKey)) { i = pid.getDouble(sPidfIKey); }
                    if(pid.has(sPidfDKey)) { d = pid.getDouble(sPidfDKey); }
                    if(pid.has(sPidfFKey)) { f = pid.getDouble(sPidfFKey); }
                    if(pid.has(sPidfTKey)) { t = pid.getDouble(sPidfTKey); }
                }
                FollowerConstants.secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(p,i,d,t,f);

                FollowerConstants.secondaryDrivePIDFFeedForward  = 0.01;
                if(follower.has(sSecondDrivePidfFFKey)) {
                    FollowerConstants.secondaryDrivePIDFFeedForward  = follower.getDouble(sSecondDrivePidfFFKey);
                }

                FollowerConstants.drivePIDFSwitch  = 20;
                if(follower.has(sDrivePidfSwitch)) {
                    FollowerConstants.drivePIDFFeedForward  = follower.getDouble(sDrivePidfSwitch);
                }

                FollowerConstants.useBrakeModeInTeleOp = false;
                FollowerConstants.automaticHoldEnd = true;
                if(follower.has(sShallHoldAtEndKey)) {
                    FollowerConstants.automaticHoldEnd  = follower.getBoolean(sShallHoldAtEndKey);
                }

                FollowerConstants.useVoltageCompensationInAuto = false;
                FollowerConstants.useVoltageCompensationInTeleOp = false;
                if(follower.has(sShallUSeVoltageCompensationKey)) {
                    FollowerConstants.useVoltageCompensationInAuto  = follower.getBoolean(sShallUSeVoltageCompensationKey);
                    FollowerConstants.useVoltageCompensationInTeleOp  = follower.getBoolean(sShallUSeVoltageCompensationKey);
                }

                FollowerConstants.nominalVoltage = 12.0;
                if(follower.has(sNominalVoltageKey)) {
                    FollowerConstants.nominalVoltage  = follower.getDouble(sNominalVoltageKey);
                }

                FollowerConstants.cacheInvalidateSeconds = 0.5;
                if(follower.has(sCacheInvalidateSecondsKey)) {
                    FollowerConstants.cacheInvalidateSeconds  = follower.getDouble(sCacheInvalidateSecondsKey);
                }

            }



        } catch( JSONException e) {
            mLogger.error(e.getMessage());
        }

        if(mLeftFront == null) {
            mLogger.error("Missing left front wheel motor in drive train configuration");
            mConfigurationValid = false;
        }
        if(mLeftBack == null) {
            mLogger.error("Missing left back wheel motor in drive train configuration");
            mConfigurationValid = false;
        }
        if(mRightFront == null) {
            mLogger.error("Missing right front wheel motor in drive train configuration");
            mConfigurationValid = false;
        }
        if(mRightBack == null) {
            mLogger.error("Missing right back wheel motor in drive train configuration");
            mConfigurationValid = false;
        }
        if(mLocalizer == null) {
            mLogger.error("Missing odometer in drive train configuration");
            mConfigurationValid = false;
        }

        if(mConfigurationValid) { initialize(); }

    }

    /**
     * Writes the current drive train configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    public void                         write(JSONObject writer) {

        if(mConfigurationValid) {
            try {

                writer.put(sTypeKey, "mecanum-drive");

                writer.put(sShortNameKey,mShortName);

                if(mDrivingMode == Mode.FIELD_CENTRIC) { writer.put(sReferenceKey,sFieldCentricKey); }
                if(mDrivingMode == Mode.ROBOT_CENTRIC) { writer.put(sReferenceKey,sRobotCentricKey); }

                JSONObject motors = new JSONObject();
                motors.put(sFrontLeftKey,mLeftFrontHwName);
                motors.put(sBackLeftKey,mLeftBackHwName);
                motors.put(sFrontRightKey,mRightFrontHwName);
                motors.put(sBackRightKey,mRightBackHwName);
                writer.put(sMotorsKey, motors);

                writer.put(sLocalizerKey, mLocalizerHwName);

                JSONObject follower = new JSONObject();
                follower.put(sMotorCachingThresholdKey,FollowerConstants.motorCachingThreshold);
                follower.put(sXMovementKey,FollowerConstants.xMovement);
                follower.put(sYMovementKey,FollowerConstants.yMovement);
                follower.put(sMaxPowerKey,FollowerConstants.maxPower);

                JSONObject pid = new JSONObject();
                pid.put(sPidfPKey,FollowerConstants.translationalPIDFCoefficients.P);
                pid.put(sPidfIKey,FollowerConstants.translationalPIDFCoefficients.I);
                pid.put(sPidfDKey,FollowerConstants.translationalPIDFCoefficients.D);
                pid.put(sPidfFKey,FollowerConstants.translationalPIDFCoefficients.F);
                follower.put(sTranslationPidfKey, pid);

                pid = new JSONObject();
                pid.put(sPidfPKey,FollowerConstants.translationalIntegral.P);
                pid.put(sPidfIKey,FollowerConstants.translationalIntegral.I);
                pid.put(sPidfDKey,FollowerConstants.translationalIntegral.D);
                pid.put(sPidfFKey,FollowerConstants.translationalIntegral.F);
                follower.put(sTranslationIntegralKey, pid);

                follower.put(sTranslationPidfFFKey, FollowerConstants.translationalPIDFFeedForward);

                pid = new JSONObject();
                pid.put(sPidfPKey,FollowerConstants.headingPIDFCoefficients.P);
                pid.put(sPidfIKey,FollowerConstants.headingPIDFCoefficients.I);
                pid.put(sPidfDKey,FollowerConstants.headingPIDFCoefficients.D);
                pid.put(sPidfFKey,FollowerConstants.headingPIDFCoefficients.F);
                follower.put(sHeadingPidfKey, pid);

                follower.put(sHeadingPidfFFKey, FollowerConstants.headingPIDFFeedForward);

                pid = new JSONObject();
                pid.put(sPidfPKey,FollowerConstants.drivePIDFCoefficients.P);
                pid.put(sPidfIKey,FollowerConstants.drivePIDFCoefficients.I);
                pid.put(sPidfDKey,FollowerConstants.drivePIDFCoefficients.D);
                pid.put(sPidfFKey,FollowerConstants.drivePIDFCoefficients.F);
                pid.put(sPidfTKey,FollowerConstants.drivePIDFCoefficients.T);
                follower.put(sDrivePidfKey, pid);

                follower.put(sDrivePidfFFKey, FollowerConstants.drivePIDFFeedForward);

                JSONObject kalman = new JSONObject();
                kalman.put(sKalmanModelKey, FollowerConstants.driveKalmanFilterParameters.modelCovariance);
                kalman.put(sKalmanDataKey, FollowerConstants.driveKalmanFilterParameters.dataCovariance);
                follower.put(sDriveKalmanFilterKey, kalman);

                follower.put(sMassKey, FollowerConstants.mass);
                follower.put(sCentripetalScalingKey, FollowerConstants.centripetalScaling);
                follower.put(sForward0PowerAccKey, FollowerConstants.forwardZeroPowerAcceleration);
                follower.put(sStrafe0PowerAccKey, FollowerConstants.lateralZeroPowerAcceleration);
                follower.put(s0PowerAccMultiplierKey, FollowerConstants.zeroPowerAccelerationMultiplier);
                follower.put(sPathEndVelocityConstKey, FollowerConstants.pathEndVelocityConstraint);
                follower.put(sPathEndTranslationConstKey, FollowerConstants.pathEndTranslationalConstraint);
                follower.put(sPathEndHeadingConstKey, FollowerConstants.pathEndHeadingConstraint);
                follower.put(sPathEndValueConstKey, FollowerConstants.pathEndTValueConstraint);
                follower.put(sPathEndTimeOutConstKey, FollowerConstants.pathEndTimeoutConstraint);
                follower.put(sApproximationSteps, FollowerConstants.APPROXIMATION_STEPS);
                follower.put(sHoldPointTranslationScalingKey, FollowerConstants.holdPointTranslationalScaling);
                follower.put(sHoldPointHeadingScalingKey, FollowerConstants.holdPointHeadingScaling);
                follower.put(sAverageVelocitySampleKey, FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER);
                follower.put(sBezierCurveSearchLimitKey, FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT);

                if(FollowerConstants.useSecondaryTranslationalPID) {
                    pid = new JSONObject();
                    pid.put(sPidfPKey,FollowerConstants.secondaryTranslationalPIDFCoefficients.P);
                    pid.put(sPidfIKey,FollowerConstants.secondaryTranslationalPIDFCoefficients.I);
                    pid.put(sPidfDKey,FollowerConstants.secondaryTranslationalPIDFCoefficients.D);
                    pid.put(sPidfFKey,FollowerConstants.secondaryTranslationalPIDFCoefficients.F);
                    follower.put(sSecondTranslationPidfKey, pid);

                    pid = new JSONObject();
                    pid.put(sPidfPKey,FollowerConstants.secondaryTranslationalIntegral.P);
                    pid.put(sPidfIKey,FollowerConstants.secondaryTranslationalIntegral.I);
                    pid.put(sPidfDKey,FollowerConstants.secondaryTranslationalIntegral.D);
                    pid.put(sPidfFKey,FollowerConstants.secondaryTranslationalIntegral.F);
                    follower.put(sSecondTranslationIntegralKey, pid);

                    follower.put(sSecondTranslationPidfFFKey, FollowerConstants.secondaryTranslationalPIDFFeedForward);
                    follower.put(sTranslationPidfSwitch,FollowerConstants.translationalPIDFSwitch);

                }

                if(FollowerConstants.useSecondaryHeadingPID) {
                    pid = new JSONObject();
                    pid.put(sPidfPKey,FollowerConstants.secondaryHeadingPIDFCoefficients.P);
                    pid.put(sPidfIKey,FollowerConstants.secondaryHeadingPIDFCoefficients.I);
                    pid.put(sPidfDKey,FollowerConstants.secondaryHeadingPIDFCoefficients.D);
                    pid.put(sPidfFKey,FollowerConstants.secondaryHeadingPIDFCoefficients.F);
                    follower.put(sSecondHeadingPidfKey, pid);

                    follower.put(sSecondHeadingPidfFFKey, FollowerConstants.secondaryHeadingPIDFFeedForward);
                    follower.put(sHeadingPidfSwitch,FollowerConstants.headingPIDFSwitch);

                }

                if(FollowerConstants.useSecondaryDrivePID) {
                    pid = new JSONObject();
                    pid.put(sPidfPKey, FollowerConstants.secondaryDrivePIDFCoefficients.P);
                    pid.put(sPidfIKey, FollowerConstants.secondaryDrivePIDFCoefficients.I);
                    pid.put(sPidfDKey, FollowerConstants.secondaryDrivePIDFCoefficients.D);
                    pid.put(sPidfFKey, FollowerConstants.secondaryDrivePIDFCoefficients.F);
                    pid.put(sPidfTKey, FollowerConstants.secondaryDrivePIDFCoefficients.T);
                    follower.put(sSecondDrivePidfKey, pid);

                    follower.put(sSecondDrivePidfFFKey, FollowerConstants.secondaryDrivePIDFFeedForward);
                    follower.put(sDrivePidfSwitch,FollowerConstants.drivePIDFSwitch);
                }

                follower.put(sShallHoldAtEndKey,FollowerConstants.automaticHoldEnd);
                follower.put(sShallUSeVoltageCompensationKey,FollowerConstants.useVoltageCompensationInAuto);
                follower.put(sNominalVoltageKey,FollowerConstants.nominalVoltage);
                follower.put(sCacheInvalidateSecondsKey,FollowerConstants.cacheInvalidateSeconds);

                writer.put(sFollowerKey,follower);


            } catch( JSONException e) {
                mLogger.error(e.getMessage());
            }
        }
    }

    /**
     * Generates an HTML representation of the drive train configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted drive train configuration.
     */
    public String                       logConfigurationHTML() {

        // Log short name
        String result = "<p style=\"padding-left:10px; font-size: 10px;\"> <span style=\"font-weight: 500\"> SHORT : </span>" +
                mShortName +
                "</p>\n" +

                "<p style=\"padding-left:10px; font-size: 10px;\"> <span style=\"font-weight: 500\"> MODE : </span>" +
                mDrivingMode +
                "</p>\n" +

                // Log motors
                "<details style=\"margin-left:10px\">\n" +
                "<summary style=\"font-size: 10px; font-weight: 500\"> MOTORS </summary>\n" +
                "<ul>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Left front wheel : " +
                mLeftFrontHwName +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Left back wheel : " +
                mLeftBackHwName +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Right front wheel : " +
                mRightFrontHwName +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Right back wheel : " +
                mRightBackHwName +
                "</li>\n" +
                "</ul>\n" +
                "</details>\n" +

                // Log localizer
                "<p style=\"padding-left:10px; font-size: 10px;\"> <span style=\"font-weight: 500\"> LOCALIZER : </span>" +
                mLocalizerHwName +
                "</p>\n" +

                // Log physics
                "<details style=\"margin-left:10px\">\n" +
                "<summary style=\"font-size: 10px; font-weight: 500\"> FOLLOWER </summary>\n" +
                "<ul>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Motor caching threshold : " +
                FollowerConstants.motorCachingThreshold +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> X Movement : " +
                FollowerConstants.xMovement +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Y Movement : " +
                FollowerConstants.yMovement +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Max power : " +
                FollowerConstants.maxPower +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Translation PIDF : P : " +
                FollowerConstants.translationalPIDFCoefficients.P +
                " I : " +
                FollowerConstants.translationalPIDFCoefficients.I +
                " D : " +
                FollowerConstants.translationalPIDFCoefficients.D +
                " F : " +
                FollowerConstants.translationalPIDFCoefficients.F +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Translation Integral PIDF : P : " +
                FollowerConstants.translationalIntegral.P +
                " I : " +
                FollowerConstants.translationalIntegral.I +
                " D : " +
                FollowerConstants.translationalIntegral.D +
                " F : " +
                FollowerConstants.translationalIntegral.F +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Translation PIDF Feed Forward : " +
                FollowerConstants.translationalPIDFFeedForward +
                "</li>\n" +
                 "<li style=\"padding-left:10px; font-size: 10px\"> Heading PIDF : P : " +
                FollowerConstants.headingPIDFCoefficients.P +
                " I : " +
                FollowerConstants.headingPIDFCoefficients.I +
                " D : " +
                FollowerConstants.headingPIDFCoefficients.D +
                " F : " +
                FollowerConstants.headingPIDFCoefficients.F +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Heading PIDF Feed Forward : " +
                FollowerConstants.headingPIDFFeedForward +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Drive PIDF : P : " +
                FollowerConstants.drivePIDFCoefficients.P +
                " I : " +
                FollowerConstants.drivePIDFCoefficients.I +
                " D : " +
                FollowerConstants.drivePIDFCoefficients.D +
                " F : " +
                FollowerConstants.drivePIDFCoefficients.F +
                " T : " +
                FollowerConstants.drivePIDFCoefficients.T +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Drive PIDF Feed Forward : " +
                FollowerConstants.drivePIDFFeedForward +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Drive Kalman : model : " +
                FollowerConstants.driveKalmanFilterParameters.modelCovariance +
                " data : " +
                FollowerConstants.driveKalmanFilterParameters.dataCovariance +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Mass : " +
                FollowerConstants.mass +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Centripetal scaling : " +
                FollowerConstants.centripetalScaling +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Forward 0 power acceleration : " +
                FollowerConstants.forwardZeroPowerAcceleration +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Lateral 0 power acceleration : " +
                FollowerConstants.lateralZeroPowerAcceleration +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> 0 power acceleration multiplier : " +
                FollowerConstants.zeroPowerAccelerationMultiplier +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Path end velocity constraint : " +
                FollowerConstants.pathEndVelocityConstraint +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Path end translation constraint : " +
                FollowerConstants.pathEndTranslationalConstraint +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Path end heading constraint : " +
                FollowerConstants.pathEndHeadingConstraint +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Path end value constraint : " +
                FollowerConstants.pathEndTValueConstraint +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Path end time out constraint : " +
                FollowerConstants.pathEndTimeoutConstraint +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Approximation steps : " +
                FollowerConstants.APPROXIMATION_STEPS +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Hold point translation scaling : " +
                FollowerConstants.holdPointTranslationalScaling +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Hold point heading scaling : " +
                FollowerConstants.holdPointHeadingScaling +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Averaged velocity samples number : " +
                FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Bezier curve search limit : " +
                FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT +
                "</li>\n";


        if(FollowerConstants.useSecondaryTranslationalPID) {

            result += "<li style=\"padding-left:10px; font-size: 10px\"> Second translation PIDF : P : " +
                    FollowerConstants.secondaryTranslationalPIDFCoefficients.P +
                    " I : " +
                    FollowerConstants.secondaryTranslationalPIDFCoefficients.I +
                    " D : " +
                    FollowerConstants.secondaryTranslationalPIDFCoefficients.D +
                    " F : " +
                    FollowerConstants.secondaryTranslationalPIDFCoefficients.F +
                    "</li>\n" +
                    "<li style=\"padding-left:10px; font-size: 10px\"> Second translation Integral PIDF : P : " +
                    FollowerConstants.secondaryTranslationalIntegral.P +
                    " I : " +
                    FollowerConstants.secondaryTranslationalIntegral.I +
                    " D : " +
                    FollowerConstants.secondaryTranslationalIntegral.D +
                    " F : " +
                    FollowerConstants.secondaryTranslationalIntegral.F +
                    "</li>\n" +
                    "<li style=\"padding-left:10px; font-size: 10px\"> Second translation PIDF Feed Forward : " +
                    FollowerConstants.secondaryTranslationalPIDFFeedForward +
                    "</li>\n" +
                    "<li style=\"padding-left:10px; font-size: 10px\"> Translation PIDF Switch : " +
                    FollowerConstants.translationalPIDFSwitch +
                    "</li>\n";

        }

        if(FollowerConstants.useSecondaryHeadingPID) {
            result += "<li style=\"padding-left:10px; font-size: 10px\"> Second heading PIDF : P : " +
                    FollowerConstants.secondaryHeadingPIDFCoefficients.P +
                    " I : " +
                    FollowerConstants.secondaryHeadingPIDFCoefficients.I +
                    " D : " +
                    FollowerConstants.secondaryHeadingPIDFCoefficients.D +
                    " F : " +
                    FollowerConstants.secondaryHeadingPIDFCoefficients.F +
                    "</li>\n" +
                    "<li style=\"padding-left:10px; font-size: 10px\"> Second heading PIDF Feed Forward : " +
                    FollowerConstants.secondaryHeadingPIDFFeedForward +
                    "</li>\n" +
                    "<li style=\"padding-left:10px; font-size: 10px\"> Heading PIDF Switch : " +
                    FollowerConstants.headingPIDFSwitch +
                    "</li>\n";

        }

        if(FollowerConstants.useSecondaryDrivePID) {

            result += "<li style=\"padding-left:10px; font-size: 10px\"> Second drive PIDF : P : " +
                    FollowerConstants.secondaryDrivePIDFCoefficients.P +
                    " I : " +
                    FollowerConstants.secondaryDrivePIDFCoefficients.I +
                    " D : " +
                    FollowerConstants.secondaryDrivePIDFCoefficients.D +
                    " F : " +
                    FollowerConstants.secondaryDrivePIDFCoefficients.F +
                    " T : " +
                    FollowerConstants.secondaryDrivePIDFCoefficients.T +
                    "</li>\n" +
                    "<li style=\"padding-left:10px; font-size: 10px\"> Second drive PIDF Feed Forward : " +
                    FollowerConstants.secondaryDrivePIDFFeedForward +
                    "</li>\n" +
                    "<li style=\"padding-left:10px; font-size: 10px\"> Drive PIDF Switch : " +
                    FollowerConstants.drivePIDFSwitch +
                    "</li>\n";

        }

        result += "<li style=\"padding-left:10px; font-size: 10px\"> Automatic hold at end : " +
                FollowerConstants.automaticHoldEnd +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Use Voltage Compensation in auto : " +
                FollowerConstants.useVoltageCompensationInAuto +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Use Voltage Compensation in teleop : " +
                FollowerConstants.useVoltageCompensationInTeleOp +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Nominal mVoltage : " +
                FollowerConstants.nominalVoltage +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Cache invalidate seconds : " +
                FollowerConstants.cacheInvalidateSeconds +
                "</li>\n" +
                "</ul>\n" +
                "</details>\n";



        return result;

    }

    /**
     * Generates a text-based representation of the drive train configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted drive train configuration details.
     */
    public String                       logConfigurationText(String header) {


        String result = header +
                "> SHORT : " +
                mShortName +
                "\n" +

                // Log motors
                header +
                "> MOTORS\n" +
                header +
                "--> Left front wheel : " +
                mLeftFrontHwName +
                "\n" +
                header +
                "--> Left back wheel : " +
                mLeftBackHwName +
                "\n" +
                header +
                "--> Right front wheel : " +
                mRightFrontHwName +
                "\n" +
                header +
                "--> Right back wheel : " +
                mRightBackHwName +
                "\n" +


                // Log localizer
                header +
                " LOCALIZER : " +
                mLocalizerHwName +
                "\n" +

                // Log physics
                header +
                "> FOLLOWER\n" +
                header +
                "--> Motor caching threshold : " +
                FollowerConstants.motorCachingThreshold +
                "\n" +
                header +
                "--> X Movement : " +
                FollowerConstants.xMovement +
                "\n" +
                header +
                "--> Y Movement : " +
                FollowerConstants.yMovement +
                "\n" +
                header +
                "--> Max power : " +
                FollowerConstants.maxPower +
                "\n" +
                header +
                "--> Translation PIDF : \n" +
                header +
                "----> P : " +
                FollowerConstants.translationalPIDFCoefficients.P +
                "\n" +
                header +
                "----> I : " +
                FollowerConstants.translationalPIDFCoefficients.I +
                "\n" +
                header +
                "----> D : " +
                FollowerConstants.translationalPIDFCoefficients.D +
                "\n" +
                header +
                "----> F : " +
                FollowerConstants.translationalPIDFCoefficients.F +
                "\n" +
                header +
                "--> Translation Integral PIDF : \n" +
                header +
                "----> P : " +
                FollowerConstants.translationalIntegral.P +
                "\n" +
                header +
                "----> I : " +
                FollowerConstants.translationalIntegral.I +
                "\n" +
                header +
                "----> D : " +
                FollowerConstants.translationalIntegral.D +
                "\n" +
                header +
                "----> F : " +
                FollowerConstants.translationalIntegral.F +
                "\n" +
                header +
                "--> Translation PIDF Feed forward : " +
                FollowerConstants.translationalPIDFFeedForward +
                "\n" +
                header +
                "--> Heading PIDF : \n" +
                header +
                "----> P : " +
                FollowerConstants.headingPIDFCoefficients.P +
                "\n" +
                header +
                "----> I : " +
                FollowerConstants.headingPIDFCoefficients.I +
                "\n" +
                header +
                "----> D : " +
                FollowerConstants.headingPIDFCoefficients.D +
                "\n" +
                header +
                "----> F : " +
                FollowerConstants.headingPIDFCoefficients.F +
                "\n" +
                header +
                "--> Heading PIDF Feed forward : " +
                FollowerConstants.headingPIDFFeedForward +
                "\n" +
                header +
                "--> Drive PIDF : \n" +
                header +
                "----> P : " +
                FollowerConstants.drivePIDFCoefficients.P +
                "\n" +
                header +
                "----> I : " +
                FollowerConstants.drivePIDFCoefficients.I +
                "\n" +
                header +
                "----> D : " +
                FollowerConstants.drivePIDFCoefficients.D +
                "\n" +
                header +
                "----> F : " +
                FollowerConstants.drivePIDFCoefficients.F +
                "\n" +
                header +
                "--> Drive PIDF Feed forward : " +
                FollowerConstants.drivePIDFFeedForward +
                "\n" +
                header +
                "--> Drive Kalman : \n" +
                header +
                "----> Model : " +
                FollowerConstants.driveKalmanFilterParameters.modelCovariance +
                "\n" +
                header +
                "----> Data : " +
                FollowerConstants.driveKalmanFilterParameters.dataCovariance +
                "\n" +
                header +
                "--> Mass : " +
                FollowerConstants.mass +
                "\n" +
                header +
                "--> Centripetal scaling : " +
                FollowerConstants.centripetalScaling +
                "\n" +
                header +
                "--> Forward 0 power acceleration : " +
                FollowerConstants.forwardZeroPowerAcceleration +
                "\n" +
                header +
                "--> Lateral 0 power acceleration : " +
                FollowerConstants.lateralZeroPowerAcceleration +
                "\n" +
                header +
                "--> 0 power acceleration multiplier : " +
                FollowerConstants.zeroPowerAccelerationMultiplier +
                "\n" +
                header +
                "--> Path end velocity constraint : " +
                FollowerConstants.pathEndVelocityConstraint +
                "\n" +
                header +
                "--> Path end translation constraint : " +
                FollowerConstants.pathEndTranslationalConstraint +
                "\n" +
                header +
                "--> Path end heading constraint : " +
                FollowerConstants.pathEndHeadingConstraint +
                "\n" +
                header +
                "--> Path end value constraint : " +
                FollowerConstants.pathEndTValueConstraint +
                "\n" +
                header +
                "--> Path end time out constraint : " +
                FollowerConstants.pathEndTimeoutConstraint +
                "\n" +
                header +
                "--> Approximation steps : " +
                FollowerConstants.APPROXIMATION_STEPS +
                "\n" +
                header +
                "--> Hold point translation scaling : " +
                FollowerConstants.holdPointTranslationalScaling +
                "\n" +
                header +
                "--> Hold point heading scaling : " +
                FollowerConstants.holdPointHeadingScaling +
                "\n" +
                header +
                "--> Averaged velocity samples number : " +
                FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER +
                "\n" +
                header +
                "--> Bezier curve search limit : " +
                FollowerConstants.BEZIER_CURVE_SEARCH_LIMIT +
                "\n";

        if(FollowerConstants.useSecondaryTranslationalPID) {

            result += header +
                    "--> Second translation PIDF : P : " +
                    FollowerConstants.secondaryTranslationalPIDFCoefficients.P +
                    " I : " +
                    FollowerConstants.secondaryTranslationalPIDFCoefficients.I +
                    " D : " +
                    FollowerConstants.secondaryTranslationalPIDFCoefficients.D +
                    " F : " +
                    FollowerConstants.secondaryTranslationalPIDFCoefficients.F +
                    "\n" +
                    header +
                    "--> Second translation Integral PIDF : P : " +
                    FollowerConstants.secondaryTranslationalIntegral.P +
                    " I : " +
                    FollowerConstants.secondaryTranslationalIntegral.I +
                    " D : " +
                    FollowerConstants.secondaryTranslationalIntegral.D +
                    " F : " +
                    FollowerConstants.secondaryTranslationalIntegral.F +
                    "\n" +
                    header +
                    "--> Second translation PIDF Feed Forward : " +
                    FollowerConstants.secondaryTranslationalPIDFFeedForward +
                    "\n" +
                    header +
                    "--> Translation PIDF Switch : " +
                    FollowerConstants.translationalPIDFSwitch +
                    "\n";

        }

        if(FollowerConstants.useSecondaryHeadingPID) {
            result += header +
                    "--> Second heading PIDF : P : " +
                    FollowerConstants.secondaryHeadingPIDFCoefficients.P +
                    " I : " +
                    FollowerConstants.secondaryHeadingPIDFCoefficients.I +
                    " D : " +
                    FollowerConstants.secondaryHeadingPIDFCoefficients.D +
                    " F : " +
                    FollowerConstants.secondaryHeadingPIDFCoefficients.F +
                    "\n" +
                    header +
                    "--> Second heading PIDF Feed Forward : " +
                    FollowerConstants.secondaryHeadingPIDFFeedForward +
                    "\n" +
                    "--> Heading PIDF Switch : " +
                    FollowerConstants.headingPIDFSwitch +
                    "\n";

        }

        if(FollowerConstants.useSecondaryDrivePID) {

            result += "--> Second drive PIDF : P : " +
                    FollowerConstants.secondaryDrivePIDFCoefficients.P +
                    " I : " +
                    FollowerConstants.secondaryDrivePIDFCoefficients.I +
                    " D : " +
                    FollowerConstants.secondaryDrivePIDFCoefficients.D +
                    " F : " +
                    FollowerConstants.secondaryDrivePIDFCoefficients.F +
                    " T : " +
                    FollowerConstants.secondaryDrivePIDFCoefficients.T +
                    "\n" +
                    header +
                    "--> Second drive PIDF Feed Forward : " +
                    FollowerConstants.secondaryDrivePIDFFeedForward +
                    "\n" +
                    header +
                    "--> Drive PIDF Switch : " +
                    FollowerConstants.drivePIDFSwitch +
                    "\n";

        }

        result += header +
                "--> Automatic hold at end : " +
                FollowerConstants.automaticHoldEnd +
                "\n" +
                header +
                "--> Use Voltage Compensation in auto : " +
                FollowerConstants.useVoltageCompensationInAuto +
                "\n" +
                header +
                "--> Use Voltage Compensation in teleop : " +
                FollowerConstants.useVoltageCompensationInTeleOp +
                "\n" +
                header +
                "--> Nominal mVoltage : " +
                FollowerConstants.nominalVoltage +
                "\n" +
                header +
                "--> Cache invalidate seconds : " +
                FollowerConstants.cacheInvalidateSeconds +
                "\n";

        return result;

    }

}


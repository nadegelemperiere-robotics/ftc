/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Mecanum Drive management
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.subsystems;

/* System includes */
import java.util.Map;

/* JSON object */
import org.json.JSONException;
import org.json.JSONObject;


/* Pedro Pathing includes */
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
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
    String                          mLocalizerHwName;

    final Hardware                  mHardware;
    LocalizerComponent              mLocalizer;

    double                          mDrivingSpeedMultiplier;
    Mode                            mDrivingMode;

    /**
     * Constructor
     * @param name Name of the drive train
     * @param hardware List of registered hardware to use
     * @param logger Logger for trace
     */
    public  MecanumDrive(String name, Hardware hardware, LogManager logger) {
        super(hardware);

        mLogger                 = logger;
        mConfigurationValid     = false;

        mDrivingMode            = Mode.ROBOT_CENTRIC;
        mDrivingSpeedMultiplier = 1.0;

        mName               = name;
        mShortName          = "";
        mLocalizerHwName    = "";

        mHardware           = hardware;
        mLocalizer          = null;
        
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

            update();

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
            Pose current = this.poseUpdater.getPose();
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
     * Determines if the actuator subsystem is configured correctly.
     *
     * @return True if the actuator is configured, false otherwise.
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
                    FollowerConstants.leftFrontMotorName = wheels.getString(sFrontLeftKey);
                    if (motors.containsKey(FollowerConstants.leftFrontMotorName)) {
                        MotorComponent temp = motors.get(FollowerConstants.leftFrontMotorName);
                        if(temp != null) {
                            FollowerConstants.leftFrontMotorDirection = temp.getDirection();
                        }
                        else {
                            mLogger.error("Missing left front wheel motor in drive train configuration");
                            mConfigurationValid = false;
                        }
                    }
                }
                if(wheels.has(sBackLeftKey)) {
                    FollowerConstants.leftRearMotorName = wheels.getString(sBackLeftKey);
                    if (motors.containsKey(FollowerConstants.leftRearMotorName)) {
                        MotorComponent temp = motors.get(FollowerConstants.leftRearMotorName);
                        if(temp != null) {
                            FollowerConstants.leftRearMotorDirection = temp.getDirection();
                        }
                        else {
                            mLogger.error("Missing left back wheel motor in drive train configuration");
                            mConfigurationValid = false;
                        }
                    }
                }
                if(wheels.has(sFrontRightKey)) {
                    FollowerConstants.rightFrontMotorName = wheels.getString(sFrontRightKey);
                    if (motors.containsKey(FollowerConstants.rightFrontMotorName)) {
                        MotorComponent temp = motors.get(FollowerConstants.rightFrontMotorName);
                        if(temp != null) {
                            FollowerConstants.rightFrontMotorDirection = temp.getDirection();
                        }
                        else {
                            mLogger.error("Missing right front wheel motor in drive train configuration");
                            mConfigurationValid = false;
                        }
                    }
                }
                if(wheels.has(sBackRightKey)) {
                    FollowerConstants.rightRearMotorName = wheels.getString(sBackRightKey);
                    if (motors.containsKey(FollowerConstants.rightRearMotorName)) {
                        MotorComponent temp = motors.get(FollowerConstants.rightRearMotorName);
                        if(temp != null) {
                            FollowerConstants.rightRearMotorDirection = temp.getDirection();
                        }
                        else {
                            mLogger.error("Missing right back wheel motor in drive train configuration");
                            mConfigurationValid = false;
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
        if(mLocalizer == null) {
            mLogger.error("Missing odometer in drive train configuration");
            mConfigurationValid = false;
        }

        if(mConfigurationValid) { initialize(mLocalizer); }

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
                motors.put(sFrontLeftKey,FollowerConstants.leftFrontMotorName);
                motors.put(sBackLeftKey,FollowerConstants.leftRearMotorName);
                motors.put(sFrontRightKey,FollowerConstants.rightFrontMotorName);
                motors.put(sBackRightKey,FollowerConstants.rightRearMotorName);
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
                FollowerConstants.leftFrontMotorName +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Left back wheel : " +
                FollowerConstants.leftRearMotorName +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Right front wheel : " +
                FollowerConstants.rightFrontMotorName +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Right back wheel : " +
                FollowerConstants.rightRearMotorName +
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
                FollowerConstants.leftFrontMotorName +
                "\n" +
                header +
                "--> Left back wheel : " +
                FollowerConstants.leftRearMotorName +
                "\n" +
                header +
                "--> Right front wheel : " +
                FollowerConstants.rightFrontMotorName +
                "\n" +
                header +
                "--> Right back wheel : " +
                FollowerConstants.rightRearMotorName +
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

    /**
     * Override the initializatiom function which should only be called in default constructor
     * of the mother class so that it doesn't attempt anything before read has occured
     */
    @Override
    public void                         initialize() { }

}


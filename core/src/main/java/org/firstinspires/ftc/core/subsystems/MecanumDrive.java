/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Mechanum Drive management
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.subsystems;

/* System includes */
import java.util.Map;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;

/* Android includes */
import androidx.annotation.NonNull;

/* JSON object */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/* ACME robotics includes */
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;


/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.motors.MotorComponent;
import org.firstinspires.ftc.core.components.odometers.OdometerComponent;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.InterOpMode;

public class MecanumDrive extends DriveTrain {

    enum Mode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    public final class FollowTrajectoryAction implements Action {

        public  final   TimeTrajectory  mTrajectory;
        private         double          mStartTime;

        public FollowTrajectoryAction(TimeTrajectory t) {
            mTrajectory = t;
            mStartTime = -1;

            mPlannedFinalPose = mTrajectory.get(mTrajectory.duration).value();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            boolean result = true;

            double t;
            if (mStartTime < 0) {
                mStartTime = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - mStartTime;
            }

            if (t >= mTrajectory.duration) {
                mLeftFront.power(0);
                mLeftBack.power(0);
                mRightBack.power(0);
                mRightFront.power(0);

                result = false;
            }
            else {
                Pose2dDual<Time> txWorldTarget = mTrajectory.get(t);
                mLocalizer.update();
                PoseVelocity2d robotVelRobot = mLocalizer.velocity();
                PoseVelocity2dDual<Time> command = new HolonomicController(
                       mXGain, mYGain, mHeadingGain, mXVelocityGain, mYVelocityGain, mHeadingVelocityGain
                ).compute(txWorldTarget,mLocalizer.pose(),robotVelRobot);

                MecanumKinematics.WheelVelocities<Time> wheelVels = mKinematics.inverse(command);
                double voltage = mVoltageSensor.getVoltage();
                MotorFeedforward feedforward = new MotorFeedforward(mKs, mKv / mInPerTick, mKa / mInPerTick);

                double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
                double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
                double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
                double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

                mLeftFront.power(leftFrontPower);
                mLeftBack.power(leftBackPower);
                mRightBack.power(rightBackPower);
                mRightFront.power(rightFrontPower);

                Pose2d error = txWorldTarget.value().minusExp(mLocalizer.pose());

                mLogger.debug("x : " +  mLocalizer.pose().position.x);
                mLogger.debug("y : " + mLocalizer.pose().position.y);
                mLogger.debug("heading (deg) : " + Math.toDegrees(mLocalizer.pose().heading.toDouble()));

                mLogger.debug("xError : " +  error.position.x);
                mLogger.debug("yError : " +  error.position.y);
                mLogger.debug("headingError  (deg): " +  Math.toDegrees(error.heading.toDouble()));


            }

            return result;
        }

        @Override
        public void preview(Canvas c) {
        }
    }

    public final class TurnAction implements Action {
        private final   TimeTurn mTurn;

        private double  mStartTime;

        public TurnAction(TimeTurn turn) {
            this.mTurn = turn;
            mStartTime = -1;

            mPlannedFinalPose = turn.get(turn.duration).value();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            boolean result = true;
            double t;
            if (mStartTime < 0) {
                mStartTime = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - mStartTime;
            }

            if (t >= mTurn.duration) {
                mLeftFront.power(0);
                mLeftBack.power(0);
                mRightBack.power(0);
                mRightFront.power(0);

                result =  false;
            }
            else {

                Pose2dDual<Time> txWorldTarget = mTurn.get(t);

                mLocalizer.update();
                PoseVelocity2d robotVelRobot = mLocalizer.velocity();

                PoseVelocity2dDual<Time> command = new HolonomicController(
                        mXGain, mYGain, mHeadingGain, mXVelocityGain, mYVelocityGain, mHeadingVelocityGain
                ).compute(txWorldTarget,mLocalizer.pose(),robotVelRobot);

                MecanumKinematics.WheelVelocities<Time> wheelVels = mKinematics.inverse(command);
                double voltage = mVoltageSensor.getVoltage();
                MotorFeedforward feedforward = new MotorFeedforward(mKs, mKv / mInPerTick, mKa / mInPerTick);

                double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
                double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
                double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
                double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

                mLeftFront.power(leftFrontPower);
                mLeftBack.power(leftBackPower);
                mRightBack.power(rightBackPower);
                mRightFront.power(rightFrontPower);

                Pose2d error = txWorldTarget.value().minusExp(mLocalizer.pose());

                mLogger.debug("x : " +  mLocalizer.pose().position.x);
                mLogger.debug("y : " + mLocalizer.pose().position.y);
                mLogger.debug("heading (deg) : " + Math.toDegrees(mLocalizer.pose().heading.toDouble()));

                mLogger.debug("xError : " +  error.position.x);
                mLogger.debug("yError : " +  error.position.y);
                mLogger.debug("headingError  (deg): " +  Math.toDegrees(error.heading.toDouble()));

            }

            return result;
        }

        @Override
        public void preview(Canvas c) {
        }
    }

    static final String             sFrontLeftKey               = "front-left-wheel";
    static final String             sBackLeftKey                = "back-left-wheel";
    static final String             sFrontRightKey              = "front-right-wheel";
    static final String             sBackRightKey               = "back-right-wheel";
    static final String             sInPerTickKey               = "in-per-tick";
    static final String             sLatInPerTickKey            = "lat-in-per-tick";
    static final String             sTrackWidthTicksKey         = "track-width-ticks";
    static final String             sMaxWheelVelocityKey        = "max-wheel-velocity";
    static final String             sMinProfileAccelerationKey  = "min-profile-acceleration";
    static final String             sMaxProfileAccelerationKey  = "max-profile-acceleration";
    static final String             sMaxHeadingVelocityKey      = "max-heading-velocity";
    static final String             sMaxHeadingAccelerationKey  = "max-heading-acceleration";
    static final String             sKsKey                      = "ks";
    static final String             sKvKey                      = "kv" ;
    static final String             sKaKey                      = "ka";
    static final String             sXGainKey                   = "x-gain";
    static final String             sXVelocityGainKey           = "x-velocity-gain";
    static final String             sYGainKey                   = "y-gain";
    static final String             sYVelocityGainKey           = "y-velocity-gain";
    static final String             sHeadingGainKey             = "heading-gain" ;
    static final String             sHeadingVelocityGainKey     = "heading-velocity-gain";

    static final String             sReferenceKey               = "reference";
    static final String             sRobotCentricKey            = "robot-centric";
    static final String             sFieldCentricKey            = "field-centric";
    static final String             sShortNameKey               = "short";

    final LogManager                mLogger;

    protected boolean               mConfigurationValid;
    boolean                         mHasFinished;

    final String                    mName;
    String                          mShortName;
    String                          mLeftFrontHwName;
    String                          mLeftBackHwName;
    String                          mRightFrontHwName;
    String                          mRightBackHwName;
    String                          mLocalizerHwName;

    final Hardware                  mHardware;
    MotorComponent                  mLeftFront;
    MotorComponent                  mRightFront;
    MotorComponent                  mLeftBack;
    MotorComponent                  mRightBack;
    OdometerComponent               mLocalizer;
    VoltageSensor                   mVoltageSensor;

    MecanumKinematics               mKinematics;

    double                          mInPerTick;
    double                          mLatInPerTick;
    double                          mTrackWidthTicks;
    double                          mMaxWheelVelocity;
    double                          mMinProfileAcceleration;
    double                          mMaxProfileAcceleration;
    double                          mMaxHeadingVelocity;
    double                          mMaxHeadingAcceleration;
    double                          mKs;
    double                          mKv;
    double                          mKa;
    double                          mXGain;
    double                          mXVelocityGain;
    double                          mYGain;
    double                          mYVelocityGain;
    double                          mHeadingGain;
    double                          mHeadingVelocityGain;

    double                          mDrivingSpeedMultiplier;
    Mode                            mDrivingMode;
    Pose2d                          mInitialPose;

    Pose2d                          mPlannedFinalPose;

    /**
     * Constructor
     * @param name Name of the drive train
     * @param hardware List of registered hardware to use
     * @param logger Logger for trace
     */
    public  MecanumDrive(String name, Hardware hardware, LogManager logger) {

        mLogger                 = logger;
        mConfigurationValid     = false;
        mHasFinished            = true;

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
        mKinematics         = null;
        if(mHardware != null) { mVoltageSensor = mHardware.voltageSensor(); }

        mInPerTick              = 1.0;
        mLatInPerTick           = 1.0;
        mTrackWidthTicks        = 0.0;
        mMaxWheelVelocity       = 50;
        mMinProfileAcceleration = -30;
        mMaxProfileAcceleration = 50;
        mMaxHeadingVelocity     = Math.PI;
        mMaxHeadingAcceleration = Math.PI;
        mKs                     = 0;
        mKv                     = 0;
        mKa                     = 0;
        mXGain                  = 0;
        mXVelocityGain          = 0;
        mYGain                  = 0;
        mYVelocityGain          = 0;
        mHeadingGain            = 0;
        mHeadingVelocityGain    = 0;

        // Reading initial position from interopmodes stored data if exist
        mInitialPose = new Pose2d(new Vector2d(0,0),0);
        mPlannedFinalPose = new Pose2d(new Vector2d(0,0),0);
        Object pose = InterOpMode.instance().get(mName + "-pose");
        if(pose != null) { mInitialPose = (Pose2d) pose; }

    }

    /**
     * Current state logging function
     */
    public void                         log() {
        if(mConfigurationValid) {
            // mLeftFront.log();
            // mLeftBack.log();
            // mRightFront.log();
            // mRightBack.log();
            mLocalizer.log();

            mLogger.info(mShortName + " POS : " +
                            " x : " + (double)((int)(mLocalizer.pose().position.x * 100)) / 100 +
                            " - y : " + (double)((int)(mLocalizer.pose().position.y * 100)) / 100 +
                            " - heading : " + (int)(mLocalizer.pose().heading.toDouble() / Math.PI * 180) + " deg");
            mLogger.info(mShortName + " SPD : " +
                    " x : " + (double)((int)(mLocalizer.velocity().linearVel.x)*1000) / 1000 +
                    " - y : " + (double)((int)(mLocalizer.velocity().linearVel.y)*1000) / 1000 +
                    " - heading : " + (double)((int)(mLocalizer.velocity().angVel / Math.PI * 1800))/1000 + " deg/s");
        }
    }

    public Pose2d                       finalPlannedPose() {
        return mLocalizer.pose().inverse().times(mPlannedFinalPose);
    }

    /**
     * Position reset function
     * @param pose Current position
     */
    public void                         initialize(Pose2d pose) {
        if(mConfigurationValid) { mInitialPose = pose; }
    }

    /**
     * Current task status
     * @return true if the train is available, false if busy
     */
    public boolean                      hasFinished() { return mHasFinished; }

    /**
     * Change the power multiplier when driving
     * @param multiplier a small number for precision, a greater for speed
     */
    public void                         driveSpeedMultiplier(double multiplier) {
        if(mConfigurationValid) { mDrivingSpeedMultiplier = multiplier; }
    }

    /**
     * Update the current drive train with new data
     */
    public void                         update() {
        mLogger.debug(mName + " start");
        if(mConfigurationValid) { mLocalizer.update(); }
        mLogger.debug(mName + " stop");
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

            double vx = xSpeed;
            double vy = ySpeed;

            if (mDrivingMode == Mode.FIELD_CENTRIC) {
                Rotation2d rotation = mLocalizer.pose().heading;
                rotation = rotation.times(mInitialPose.heading);
                Vector2d robotcentric = rotation.times(new Vector2d(xSpeed, ySpeed));
                vx = robotcentric.x;
                vy = robotcentric.y;
            }

            vx *= 1.1; // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(vy) + Math.abs(vx) + Math.abs(headingSpeed), 1);
            double frontLeftPower = (vy + vx + headingSpeed) / denominator * mDrivingSpeedMultiplier;
            double backLeftPower = (vy - vx + headingSpeed) / denominator * mDrivingSpeedMultiplier;
            double frontRightPower = (vy - vx - headingSpeed) / denominator * mDrivingSpeedMultiplier;
            double backRightPower = (vy + vx - headingSpeed) / denominator * mDrivingSpeedMultiplier;

            mLeftFront.power(frontLeftPower);
            mLeftBack.power(backLeftPower);
            mRightFront.power(frontRightPower);
            mRightBack.power(backRightPower);

            mLogger.debug(LogManager.Target.FILE,"stop");

        }
    }

    public TrajectoryActionBuilder      trajectory(Pose2d pose){

        TrajectoryActionBuilder result = null;

        if(mConfigurationValid) {

            TurnConstraints turnConstraints = new TurnConstraints(
                    mMaxHeadingVelocity, -mMaxHeadingAcceleration, mMaxHeadingAcceleration);
            VelConstraint   velConstraints = new MinVelConstraint(Arrays.asList(
                    mKinematics.new WheelVelConstraint(mMaxWheelVelocity),
                    new AngularVelConstraint(mMaxHeadingVelocity)));
            AccelConstraint  accContraint =
                    new ProfileAccelConstraint(mMinProfileAcceleration, mMaxProfileAcceleration);

            result =  new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(0.25, 0.1, 1e-2)
                ), mLocalizer.pose().times(pose), 0.0,
                turnConstraints, velConstraints, accContraint
            );
        }

        return result;
    }

    /**
     * Persist data to be able to keep the same behavior after a reinitialization.
     * Read current heading and transform it into the FTC field coordinate system
     */
    public void                         persist()
    {
        if(mConfigurationValid) {
            Pose2d current = mLocalizer.pose();
            current = current.times(mInitialPose);
            InterOpMode.instance().add(mName + "-pose", current);
        }
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
        mKinematics             = null;

        mInPerTick              = 1.0;
        mLatInPerTick           = 1.0;
        mTrackWidthTicks        = 0.0;
        mMaxWheelVelocity       = 50;
        mMinProfileAcceleration = -30;
        mMaxProfileAcceleration = 50;
        mMaxHeadingVelocity     = Math.PI;
        mMaxHeadingAcceleration = Math.PI;
        mKs                     = 0;
        mKv                     = 0;
        mKa                     = 0;
        mXGain                  = 0;
        mXVelocityGain          = 0;
        mYGain                  = 0;
        mYVelocityGain          = 0;
        mHeadingGain            = 0;
        mHeadingVelocityGain    = 0;
        mDrivingMode            = Mode.ROBOT_CENTRIC;

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

            if(reader.has(sOdometerKey)) {
                Map<String,OdometerComponent> odometers = mHardware.odometers();
                mLocalizerHwName = reader.getString(sOdometerKey);
                if (odometers.containsKey(mLocalizerHwName)) {
                    mLocalizer = odometers.get(mLocalizerHwName);
                }
            }

            if(reader.has(sPhysicsKey)) {

                JSONObject physics = reader.getJSONObject(sPhysicsKey);
                if(physics.has(sInPerTickKey))              { mInPerTick = physics.getDouble(sInPerTickKey); }
                if(physics.has(sLatInPerTickKey))           { mLatInPerTick = physics.getDouble(sLatInPerTickKey); }
                if(physics.has(sTrackWidthTicksKey))        { mTrackWidthTicks = physics.getDouble(sTrackWidthTicksKey); }
                if(physics.has(sMaxWheelVelocityKey))       { mMaxWheelVelocity = physics.getDouble(sMaxWheelVelocityKey); }
                if(physics.has(sMinProfileAccelerationKey)) { mMinProfileAcceleration = physics.getDouble(sMinProfileAccelerationKey); }
                if(physics.has(sMaxProfileAccelerationKey)) { mMaxProfileAcceleration = physics.getDouble(sMaxProfileAccelerationKey); }
                if(physics.has(sMaxHeadingVelocityKey))     { mMaxHeadingVelocity = physics.getDouble(sMaxHeadingVelocityKey); }
                if(physics.has(sMaxHeadingAccelerationKey)) { mMaxHeadingAcceleration = physics.getDouble(sMaxHeadingAccelerationKey); }

            }

            if(reader.has(sPidfKey)) {

                JSONObject pidf = reader.getJSONObject(sPidfKey);
                if(pidf.has(sKsKey))                    { mKs = pidf.getDouble(sKsKey); }
                if(pidf.has(sKvKey))                    { mKv = pidf.getDouble(sKsKey); }
                if(pidf.has(sKaKey))                    { mKa = pidf.getDouble(sKsKey); }
                if(pidf.has(sXGainKey))                 { mXGain = pidf.getDouble(sXGainKey); }
                if(pidf.has(sXVelocityGainKey))         { mXVelocityGain = pidf.getDouble(sXVelocityGainKey); }
                if(pidf.has(sYGainKey))                 { mYGain = pidf.getDouble(sYGainKey); }
                if(pidf.has(sYVelocityGainKey))         { mYVelocityGain = pidf.getDouble(sYVelocityGainKey); }
                if(pidf.has(sHeadingGainKey))           { mHeadingGain = pidf.getDouble(sHeadingGainKey); }
                if(pidf.has(sHeadingVelocityGainKey))   { mHeadingVelocityGain = pidf.getDouble(sHeadingVelocityGainKey); }
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

        if(mConfigurationValid) {
            mKinematics = new MecanumKinematics(
                   mInPerTick * mTrackWidthTicks, mInPerTick / mLatInPerTick);

        }

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

                JSONObject motors = new JSONObject();
                motors.put(sFrontLeftKey,mLeftFrontHwName);
                motors.put(sBackLeftKey,mLeftBackHwName);
                motors.put(sFrontRightKey,mRightFrontHwName);
                motors.put(sBackRightKey,mRightBackHwName);
                writer.put(sMotorsKey, motors);

                writer.put(sOdometerKey, mLocalizerHwName);

                JSONObject physics = new JSONObject();
                physics.put(sInPerTickKey,mInPerTick);
                physics.put(sLatInPerTickKey,mLatInPerTick);
                physics.put(sTrackWidthTicksKey,mTrackWidthTicks);
                physics.put(sMaxWheelVelocityKey,mMaxWheelVelocity);
                physics.put(sMinProfileAccelerationKey,mMinProfileAcceleration);
                physics.put(sMaxProfileAccelerationKey,mMaxProfileAcceleration);
                physics.put(sMaxHeadingVelocityKey,mMaxHeadingVelocity);
                physics.put(sMaxHeadingAccelerationKey,mMaxHeadingAcceleration);
                writer.put(sPhysicsKey, physics);

                JSONObject pidf = new JSONObject();
                pidf.put(sKsKey, mKs);
                pidf.put(sKvKey, mKv);
                pidf.put(sKaKey, mKa);
                pidf.put(sXGainKey, mXGain);
                pidf.put(sXVelocityGainKey, mXVelocityGain);
                pidf.put(sYGainKey, mYGain);
                pidf.put(sYVelocityGainKey, mYVelocityGain);
                pidf.put(sHeadingGainKey, mHeadingGain);
                pidf.put(sHeadingVelocityGainKey, mHeadingVelocityGain);
                writer.put(sPidfKey,pidf);

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
                "<p style=\"padding-left:10px; font-size: 10px;\"> <span style=\"font-weight: 500\"> ODOMETER : </span>" +
                mLocalizerHwName +
                "</p>\n" +

                // Log physics
                "<details style=\"margin-left:10px\">\n" +
                "<summary style=\"font-size: 10px; font-weight: 500\"> PHYSICS </summary>\n" +
                "<ul>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> In per tick : " +
                mInPerTick +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Lat in per tick : " +
                mLatInPerTick +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Track width in ticks : " +
                mTrackWidthTicks +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Max wheel velocity : " +
                mMaxWheelVelocity +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Min profile acceleration : " +
                mMinProfileAcceleration +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Max profile acceleration : " +
                mMaxProfileAcceleration +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Max heading velocity : " +
                mMaxHeadingVelocity +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Max heading acceleration : " +
                mMaxHeadingAcceleration +
                "</li>\n" +
                "</ul>\n" +
                "</details>\n" +


                // Log pidf
                "<details style=\"margin-left:10px\">\n" +
                "<summary style=\"font-size: 10px; font-weight: 500\"> PIDF </summary>\n" +
                "<ul>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Ks : " +
                mKs +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Kv : " +
                mKv +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Ka : " +
                mKa +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> X Gain : " +
                mXGain +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> X velocity gain : " +
                mXVelocityGain +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Y gain : " +
                mYGain +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Y velocity gain : " +
                mYVelocityGain +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Heading gain : " +
                mHeadingGain +
                "</li>\n" +
                "<li style=\"padding-left:10px; font-size: 10px\"> Heading velocity gain : " +
                mHeadingVelocityGain +
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
                " ODOMETER : " +
                mLocalizerHwName +
                "\n" +

                // Log physics
                header +
                "> PHYSICS\n" +
                header +
                "--> In per tick : " +
                mInPerTick +
                "\n" +
                header +
                "--> Lat in per tick : " +
                mLatInPerTick +
                "\n" +
                header +
                "--> Track width in ticks : " +
                mTrackWidthTicks +
                "\n" +
                header +
                "--> Max wheel velocity : " +
                mMaxWheelVelocity +
                "\n" +
                header +
                "--> Min profile acceleration : " +
                mMinProfileAcceleration +
                "\n" +
                header +
                "--> Max profile acceleration : " +
                mMaxProfileAcceleration +
                "\n" +
                header +
                "--> Max heading velocity : " +
                mMaxHeadingVelocity +
                "\n" +
                header +
                "--> Max heading acceleration : " +
                mMaxHeadingAcceleration +
                "\n" +

                // Log pidf
                header +
                "> PIDF\n" +
                header +
                "--> Ks : " +
                mKs +
                "\n" +
                header +
                "--> Kv : " +
                mKv +
                "\n" +
                header +
                "--> Ka : " +
                mKa +
                "\n" +
                header +
                "--> X Gain : " +
                mXGain +
                "\n" +
                header +
                "--> X velocity gain : " +
                mXVelocityGain +
                "\n" +
                header +
                "--> Y gain : " +
                mYGain +
                "\n" +
                header +
                "--> Y velocity gain : " +
                mYVelocityGain +
                "\n" +
                header +
                "--> Heading gain : " +
                mHeadingGain +
                "\n" +
                header +
                "--> Heading velocity gain : " +
                mHeadingVelocityGain +
                "\n";

        return result;

    }

    /**
     * List of left motors  for tuning
     *
     * @return A list of the left motors
     */
    public List<MotorComponent>         left(){
        List<MotorComponent> result = new ArrayList<>();
        if(mConfigurationValid) {
            result.add(mLeftFront);
            result.add(mLeftBack);
        }
        return result;
    }

    /**
     * List of right motors  for tuning
     *
     * @return A list of the right motors
     */
    public List<MotorComponent>         right(){
        List<MotorComponent> result = new ArrayList<>();
        if(mConfigurationValid) {
            result.add(mRightFront);
            result.add(mRightBack);
        }
        return result;
    }
}


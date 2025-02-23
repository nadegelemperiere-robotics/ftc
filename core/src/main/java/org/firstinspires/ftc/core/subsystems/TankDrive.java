/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Tank Drive management
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.subsystems;

/* System includes */
import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.Map;
import java.util.List;
import java.util.ArrayList;

/* JSON object */
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONArray;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/* ACME robotics includes */
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.RamseteController;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.motors.MotorComponent;
import org.firstinspires.ftc.core.components.odometers.OdometerComponent;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.InterOpMode;

public class TankDrive extends DriveTrain {

    public final class FollowTrajectoryAction implements Action {

        public  final TimeTrajectory mTrajectory;
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
                for(int i_left = 0; i_left < mLeft.size();i_left ++) { mLeft.get(i_left).power(0); }
                for(int i_right = 0; i_right < mRight.size();i_right ++) { mRight.get(i_right).power(0); }

                result = false;
            }
            else {

                DualNum<Time> x = mTrajectory.profile.get(t);
                Pose2dDual<Arclength> txWorldTarget = mTrajectory.path.get(x.value(), 3);

                mLocalizer.update();
                PoseVelocity2dDual<Time> command = new RamseteController(mKinematics.trackWidth, mRamseteZeta, mRamseteBBAr)
                        .compute(x, txWorldTarget, mLocalizer.pose());

                TankKinematics.WheelVelocities<Time> wheelVels = mKinematics.inverse(command);
                double voltage = mVoltageSensor.getVoltage();
                MotorFeedforward feedforward = new MotorFeedforward(mKs, mKv / mInPerTick, mKa / mInPerTick);

                double leftPower = feedforward.compute(wheelVels.left) / voltage;
                double rightPower = feedforward.compute(wheelVels.right) / voltage;

                for(int i_left = 0; i_left < mLeft.size();i_left ++) { mLeft.get(i_left).power(leftPower); }
                for(int i_right = 0; i_right < mRight.size();i_right ++) { mRight.get(i_right).power(rightPower); }

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
        public void preview(@NonNull Canvas c) {
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn mTurn;

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
                for(int i_left = 0; i_left < mLeft.size();i_left ++) { mLeft.get(i_left).power(0); }
                for(int i_right = 0; i_right < mRight.size();i_right ++) { mRight.get(i_right).power(0); }

                result = false;
            }
            else {

                Pose2dDual<Time> txWorldTarget = mTurn.get(t);

                mLocalizer.update();
                PoseVelocity2d robotVelRobot = mLocalizer.velocity();
                PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<>(
                        Vector2dDual.constant(new Vector2d(0, 0), 3),
                        txWorldTarget.heading.velocity().plus(
                                mTurnGain * mLocalizer.pose().heading.minus(txWorldTarget.heading.value()) +
                                mTurnVelocityGain * (robotVelRobot.angVel - txWorldTarget.heading.velocity().value())
                        )
                );

                TankKinematics.WheelVelocities<Time> wheelVels = mKinematics.inverse(command);
                double voltage = mVoltageSensor.getVoltage();
                MotorFeedforward feedforward = new MotorFeedforward(mKs, mKv / mInPerTick, mKa / mInPerTick);

                double leftPower = feedforward.compute(wheelVels.left) / voltage;
                double rightPower = feedforward.compute(wheelVels.right) / voltage;

                for(int i_left = 0; i_left < mLeft.size();i_left ++) { mLeft.get(i_left).power(leftPower); }
                for(int i_right = 0; i_right < mRight.size();i_right ++) { mRight.get(i_right).power(rightPower); }

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
        public void preview(@NonNull Canvas c) {
        }
    }


    static final String sLeftKey                    = "left";
    static final String sRightKey                   = "right";
    static final String sInPerTickKey               = "in-per-tick";
    static final String sTrackWidthTicksKey         = "track-width-ticks";
    static final String sMaxWheelVelocityKey        = "max-wheel-velocity";
    static final String sMinProfileAccelerationKey  = "min-profile-acceleration";
    static final String sMaxProfileAccelerationKey  = "max-profile-acceleration";
    static final String sMaxHeadingVelocityKey      = "max-heading-velocity";
    static final String sMaxHeadingAccelerationKey  = "max-heading-acceleration";
    static final String sKsKey                      = "ks";
    static final String sKvKey                      = "kv";
    static final String sKaKey                      = "ka";
    static final String sRamseteZetaKey             = "ramsete-zeta";
    static final String sRamseteBBArKey             = "ramsete-bbar";
    static final String sTurnGainKey                = "turn-gain";
    static final String sTurnVelocityGainKey        = "turn-velocity-gain";
    static final String sShortNameKey               = "short";

    final LogManager        mLogger;

    protected boolean       mConfigurationValid;
    boolean                 mHasFinished;

    final String            mName;
    String                  mShortName;

    List<String>            mLeftHwName;
    List<String>            mRightHwName;
    String                  mLocalizerHwName;

    final Hardware          mHardware;
    List<MotorComponent>    mLeft;
    List<MotorComponent>    mRight;
    OdometerComponent       mLocalizer;
    TankKinematics          mKinematics;
    VoltageSensor           mVoltageSensor;

    double                  mInPerTick;
    double                  mTrackWidthTicks;
    double                  mMaxWheelVelocity;
    double                  mMinProfileAcceleration;
    double                  mMaxProfileAcceleration;
    double                  mMaxHeadingVelocity;
    double                  mMaxHeadingAcceleration;
    double                  mKs;
    double                  mKv;
    double                  mKa;
    double                  mRamseteZeta;
    double                  mRamseteBBAr;
    double                  mTurnGain;
    double                  mTurnVelocityGain;

    double                  mDrivingSpeedMultiplier;
    Pose2d                  mInitialPose;

    Pose2d                  mPlannedFinalPose;

    public TankDrive(String name, Hardware hardware, LogManager logger) {

        mLogger = logger;
        mConfigurationValid = false;
        mHasFinished = true;

        mDrivingSpeedMultiplier = 1.0;

        mName = name;
        mShortName = "";
        mLeftHwName = new ArrayList<>();
        mRightHwName = new ArrayList<>();
        mLocalizerHwName = "";

        mHardware = hardware;
        mLeft = new ArrayList<>();
        mRight = new ArrayList<>();
        mLocalizer = null;
        mKinematics = null;
        if(mHardware != null) { mVoltageSensor = mHardware.voltageSensor(); }

        mInPerTick = 1.0;
        mTrackWidthTicks = 0.0;
        mMaxWheelVelocity = 50;
        mMinProfileAcceleration = -30;
        mMaxProfileAcceleration = 50;
        mMaxHeadingVelocity = Math.PI;
        mMaxHeadingAcceleration = Math.PI;
        mKs = 0;
        mKv = 0;
        mKa = 0;
        mRamseteZeta = 0.7;
        mRamseteBBAr = 0.2;
        mTurnGain = 0;
        mTurnVelocityGain = 0;

        // Reading initial position from interopmodes stored data if exist
        mInitialPose = new Pose2d(new Vector2d(0,0),0);
        mPlannedFinalPose = new Pose2d(new Vector2d(0,0),0);
        Object pose = InterOpMode.instance().get(mName + "-pose");
        if(pose != null) { mInitialPose = (Pose2d) pose; }

    }

    public void                         log() {
        mLocalizer.log();
    }

    public void                         initialize(Pose2d pose) {
        if(mConfigurationValid) {
            mPlannedFinalPose = pose;
            mInitialPose = pose;
            mLocalizer.pose(pose);
        }
    }

    public boolean hasFinished() {
        return mHasFinished;
    }

    public void driveSpeedMultiplier(double multiplier) {
        mDrivingSpeedMultiplier = multiplier;
    }

    public double driveSpeedMultiplier() {
        return mDrivingSpeedMultiplier;
    }

    public Pose2d                       finalPlannedPose() {
        return mLocalizer.pose().inverse().times(mPlannedFinalPose);
    }

    public void update() {
        mLogger.debug(mName + " start");
        if(mConfigurationValid) { mLocalizer.update(); }
        mLogger.debug(mName + " stop");
    }

    @Override
    public void drive(double forwardSpeed, double nothing, double headingSpeed) {

        if (mConfigurationValid) {

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(forwardSpeed) + Math.abs(headingSpeed), 1);
            double leftPower = (forwardSpeed + headingSpeed) / denominator * mDrivingSpeedMultiplier;
            double rightPower = (headingSpeed - forwardSpeed) / denominator * mDrivingSpeedMultiplier;

            for (int i_left = 0; i_left < mLeft.size(); i_left++) {
                mLeft.get(i_left).power(leftPower);
            }
            for (int i_right = 0; i_right < mRight.size(); i_right++) {
                mRight.get(i_right).power(rightPower);
            }

        }
    }

    public TrajectoryActionBuilder trajectory(){

        TrajectoryActionBuilder result = null;

        if(mConfigurationValid) {

            TurnConstraints turnConstraints = new TurnConstraints(
                    mMaxHeadingVelocity, -mMaxHeadingAcceleration, mMaxHeadingAcceleration);
            VelConstraint velConstraints = new MinVelConstraint(Arrays.asList(
                    mKinematics.new WheelVelConstraint(mMaxWheelVelocity),
                    new AngularVelConstraint(mMaxHeadingVelocity)));
            AccelConstraint accContraint =
                    new ProfileAccelConstraint(mMinProfileAcceleration, mMaxProfileAcceleration);

            result =  new TrajectoryActionBuilder(
                    TurnAction::new,
                    FollowTrajectoryAction::new,
                    new TrajectoryBuilderParams(
                            1e-6,
                            new ProfileParams(0.25, 0.1, 1e-2)
                    ), mPlannedFinalPose, 0.0,
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
    public boolean isConfigured() {
        return mConfigurationValid;
    }

    /**
     * Reads and applies the mecanum drive configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    public void read(JSONObject reader) {

        mConfigurationValid = true;

        mLeft.clear();
        mRight.clear();
        mLeftHwName.clear();
        mRightHwName.clear();

        mInPerTick = 1.0;
        mTrackWidthTicks = 0.0;
        mMaxWheelVelocity = 50;
        mMinProfileAcceleration = -30;
        mMaxProfileAcceleration = 50;
        mMaxHeadingVelocity = Math.PI;
        mMaxHeadingAcceleration = Math.PI;
        mKs = 0;
        mKv = 0;
        mKa = 0;
        mRamseteZeta = 0.7;
        mRamseteBBAr = 0.2;
        mTurnGain = 0;
        mTurnVelocityGain = 0;

        try {

            if(reader.has(sShortNameKey)) {
                mShortName = reader.getString(sShortNameKey);
            }
            if(mShortName.isEmpty()) { mShortName = mName; }

            if (reader.has(sMotorsKey)) {
                Map<String, MotorComponent> motors = mHardware.motors();
                JSONObject wheels = reader.getJSONObject(sMotorsKey);

                if (wheels.has(sLeftKey)) {
                    JSONArray lefts = wheels.getJSONArray(sLeftKey);
                    for (int i_left = 0; i_left < lefts.length(); i_left++) {
                        if (motors.containsKey(lefts.getString(i_left))) {
                            MotorComponent motor = motors.get(lefts.getString(i_left));
                            if (motor != null) {
                                motor.mode(DcMotor.RunMode.RUN_USING_ENCODER);
                                motor.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                mLeft.add(motor);
                                mLeftHwName.add(lefts.getString(i_left));
                            }
                        }
                    }
                }

                if (wheels.has(sRightKey)) {
                    JSONArray rights = wheels.getJSONArray(sRightKey);
                    for (int i_right = 0; i_right < rights.length(); i_right++) {
                        if (motors.containsKey(rights.getString(i_right))) {
                            MotorComponent motor = motors.get(rights.getString(i_right));
                            if (motor != null) {
                                motor.mode(DcMotor.RunMode.RUN_USING_ENCODER);
                                motor.zeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                mRight.add(motor);
                                mRightHwName.add(rights.getString(i_right));
                            }
                        }
                    }
                }
            }

            if (reader.has(sOdometerKey)) {
                Map<String, OdometerComponent> odometers = mHardware.odometers();
                mLocalizerHwName = reader.getString(sOdometerKey);
                if (odometers.containsKey(mLocalizerHwName)) {
                    mLocalizer = odometers.get(mLocalizerHwName);
                }
            }

            if (reader.has(sPhysicsKey)) {

                JSONObject physics = reader.getJSONObject(sPhysicsKey);
                if (physics.has(sInPerTickKey)) {
                    mInPerTick = physics.getDouble(sInPerTickKey);
                }
                if (physics.has(sTrackWidthTicksKey)) {
                    mTrackWidthTicks = physics.getDouble(sTrackWidthTicksKey);
                }
                if (physics.has(sMaxWheelVelocityKey)) {
                    mMaxWheelVelocity = physics.getDouble(sMaxWheelVelocityKey);
                }
                if (physics.has(sMinProfileAccelerationKey)) {
                    mMinProfileAcceleration = physics.getDouble(sMinProfileAccelerationKey);
                }
                if (physics.has(sMaxProfileAccelerationKey)) {
                    mMaxProfileAcceleration = physics.getDouble(sMaxProfileAccelerationKey);
                }
                if (physics.has(sMaxHeadingVelocityKey)) {
                    mMaxHeadingVelocity = physics.getDouble(sMaxHeadingVelocityKey);
                }
                if (physics.has(sMaxHeadingAccelerationKey)) {
                    mMaxHeadingAcceleration = physics.getDouble(sMaxHeadingAccelerationKey);
                }

            }

            if (reader.has(sPidfKey)) {

                JSONObject pidf = reader.getJSONObject(sPidfKey);
                if (pidf.has(sKsKey)) {
                    mKs = pidf.getDouble(sKsKey);
                }
                if (pidf.has(sKvKey)) {
                    mKv = pidf.getDouble(sKsKey);
                }
                if (pidf.has(sKaKey)) {
                    mKa = pidf.getDouble(sKsKey);
                }
                if (pidf.has(sRamseteZetaKey)) {
                    mRamseteZeta = pidf.getDouble(sRamseteZetaKey);
                }
                if (pidf.has(sRamseteBBArKey)) {
                    mRamseteBBAr = pidf.getDouble(sRamseteBBArKey);
                }
                if (pidf.has(sTurnGainKey)) {
                    mTurnGain = pidf.getDouble(sTurnGainKey);
                }
                if (pidf.has(sTurnVelocityGainKey)) {
                    mTurnVelocityGain = pidf.getDouble(sTurnGainKey);
                }
            }

        } catch (JSONException e) {
            mLogger.error(e.getMessage());
        }

        if (mLeft.isEmpty()) {
            mLogger.error("Missing left wheel motors in drive train configuration");
            mConfigurationValid = false;
        }
        if (mRight.isEmpty()) {
            mLogger.error("Missing right wheel motors in drive train configuration");
            mConfigurationValid = false;
        }
        if (mLocalizer == null) {
            mLogger.error("Missing odometer in drive train configuration");
            mConfigurationValid = false;
        }

        if(mConfigurationValid) {
            mKinematics = new TankKinematics(
                    mInPerTick * mTrackWidthTicks);

        }

    }

    /**
     * Writes the current drive train configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    public void write(JSONObject writer) {

        if (mConfigurationValid) {
            try {

                writer.put(sTypeKey, "tank-drive");

                writer.put(sShortNameKey,mShortName);

                JSONObject motors = new JSONObject();
                JSONArray lefts = new JSONArray();
                for (int i_left = 0; i_left < mLeftHwName.size(); i_left++) {
                    lefts.put(mLeftHwName.get(i_left));
                }
                JSONArray rights = new JSONArray();
                for (int i_right = 0; i_right < mRightHwName.size(); i_right++) {
                    rights.put(mRightHwName.get(i_right));
                }
                motors.put(sLeftKey, lefts);
                motors.put(sRightKey, rights);
                writer.put(sMotorsKey, motors);

                writer.put(sOdometerKey, mLocalizerHwName);

                JSONObject physics = new JSONObject();
                physics.put(sInPerTickKey, mInPerTick);
                physics.put(sTrackWidthTicksKey, mTrackWidthTicks);
                physics.put(sMaxWheelVelocityKey, mMaxWheelVelocity);
                physics.put(sMinProfileAccelerationKey, mMinProfileAcceleration);
                physics.put(sMaxProfileAccelerationKey, mMaxProfileAcceleration);
                physics.put(sMaxHeadingVelocityKey, mMaxHeadingVelocity);
                physics.put(sMaxHeadingAccelerationKey, mMaxHeadingAcceleration);
                writer.put(sPhysicsKey, physics);

                JSONObject pidf = new JSONObject();
                pidf.put(sKsKey, mKs);
                pidf.put(sKvKey, mKv);
                pidf.put(sKaKey, mKa);
                pidf.put(sRamseteZetaKey, mRamseteZeta);
                pidf.put(sRamseteBBArKey, mRamseteBBAr);
                pidf.put(sTurnGainKey, mTurnGain);
                pidf.put(sTurnVelocityGainKey, mTurnVelocityGain);
                writer.put(sPidfKey, pidf);

            } catch (JSONException e) {
                mLogger.error(e.getMessage());
            }
        }
    }

    public String logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        // Log short name
        result.append("<p style=\"padding-left:10px; font-size: 10px;\"> <span style=\"font-weight: 500\"> SHORT : </span>")
                .append(mShortName)
                .append("</p>\n");

        // Log motors
        result.append("<details style=\"margin-left:10px\">\n")
                .append("<summary style=\"font-size: 10px; font-weight: 500\"> MOTORS </summary>\n")
                .append("<ul>\n")
                .append("<summary style=\"font-size: 10px; font-weight: 500\"> LEFT </summary>\n")
                .append("<ul>\n");

        for (int i_left = 0; i_left < mLeftHwName.size(); i_left++) {
            result.append("<li style=\"padding-left:10px; font-size: 10px\">")
                    .append(mLeftHwName.get(i_left))
                    .append("</li>\n");
        }

        result.append("</ul>\n")
                .append("</details>\n")
                .append("<summary style=\"font-size: 10px; font-weight: 500\"> RIGHT </summary>\n")
                .append("<ul>\n");

        for (int i_right = 0; i_right < mRightHwName.size(); i_right++) {
            result.append("<li style=\"padding-left:10px; font-size: 10px\">")
                    .append(mRightHwName.get(i_right))
                    .append("</li>\n");
        }

        result.append("</ul>\n")
                .append("</details>\n")
                .append("</ul>\n")
                .append("</details>\n");

        // Log localizer

        result.append("<p style=\"padding-left:10px; font-size: 10px;\"> <span style=\"font-weight: 500\"> ODOMETER : </span>")
                .append(mLocalizerHwName)
                .append("</p>\n");

        // Log physics
        result.append("<details style=\"margin-left:10px\">\n")
                .append("<summary style=\"font-size: 10px; font-weight: 500\"> PHYSICS </summary>\n")
                .append("<ul>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> In per tick : ")
                .append(mInPerTick)
                .append("</li>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> Track width in ticks : ")
                .append(mTrackWidthTicks)
                .append("</li>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> Max wheel velocity : ")
                .append(mMaxWheelVelocity)
                .append("</li>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> Min profile acceleration : ")
                .append(mMinProfileAcceleration)
                .append("</li>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> Max profile acceleration : ")
                .append(mMaxProfileAcceleration)
                .append("</li>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> Max heading velocity : ")
                .append(mMaxHeadingVelocity)
                .append("</li>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> Max heading acceleration : ")
                .append(mMaxHeadingAcceleration)
                .append("</li>\n")
                .append("</ul>\n")
                .append("</details>\n");

        // Log pidf
        result.append("<details style=\"margin-left:10px\">\n")
                .append("<summary style=\"font-size: 10px; font-weight: 500\"> PIDF </summary>\n")
                .append("<ul>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> Ks : ")
                .append(mKs)
                .append("</li>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> Kv : ")
                .append(mKv)
                .append("</li>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> Ka : ")
                .append(mKa)
                .append("</li>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> Ramsete zeta : ")
                .append(mRamseteZeta)
                .append("</li>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> Ramsete bbar : ")
                .append(mRamseteBBAr)
                .append("</li>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> Turn gain : ")
                .append(mTurnGain)
                .append("</li>\n")
                .append("<li style=\"padding-left:10px; font-size: 10px\"> Turn velocity gain : ")
                .append(mTurnVelocityGain)
                .append("</li>\n")
                .append("</ul>\n")
                .append("</details>\n");

        return result.toString();

    }

    public String logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();

        result.append(header)
                .append("> SHORT : ")
                .append(mShortName)
                .append("\n");

        // Log motors
        result.append(header)
                .append("> MOTORS\n")
                .append(header)
                .append("--> LEFT :\n");
        for (int i_left = 0; i_left < mLeftHwName.size(); i_left++) {
            result.append("---->")
                    .append(mLeftHwName.get(i_left))
                    .append("\n");
        }
        result.append(header)
                .append("--> RIGHT :\n");
        for (int i_right = 0; i_right < mRightHwName.size(); i_right++) {
            result.append("---->")
                    .append(mRightHwName.get(i_right))
                    .append("\n");
        }

        // Log localizer
        result.append(header)
                .append(" ODOMETER : ")
                .append(mLocalizerHwName)
                .append("\n");

        // Log physics
        result.append(header)
                .append("> PHYSICS\n")
                .append(header)
                .append("--> In per tick : ")
                .append(mInPerTick)
                .append("\n")
                .append(header)
                .append("--> Track width in ticks : ")
                .append(mTrackWidthTicks)
                .append("\n")
                .append(header)
                .append("--> Max wheel velocity : ")
                .append(mMaxWheelVelocity)
                .append("\n")
                .append(header)
                .append("--> Min profile acceleration : ")
                .append(mMinProfileAcceleration)
                .append("\n")
                .append(header)
                .append("--> Max profile acceleration : ")
                .append(mMaxProfileAcceleration)
                .append("\n")
                .append(header)
                .append("--> Max heading velocity : ")
                .append(mMaxHeadingVelocity)
                .append("\n")
                .append(header)
                .append("--> Max heading acceleration : ")
                .append(mMaxHeadingAcceleration)
                .append("\n");

        // Log pidf
        result.append(header)
                .append("> PIDF\n")
                .append(header)
                .append("--> Ks : ")
                .append(mKs)
                .append("\n")
                .append(header)
                .append("--> Kv : ")
                .append(mKv)
                .append("\n")
                .append(header)
                .append("--> Ka : ")
                .append(mKa)
                .append("\n")
                .append(header)
                .append("--> Ramsete Zeta : ")
                .append(mRamseteZeta)
                .append("\n")
                .append(header)
                .append("--> Ramsete BBar : ")
                .append(mRamseteBBAr)
                .append("\n")
                .append(header)
                .append("--> Turn gain : ")
                .append(mTurnGain)
                .append("\n")
                .append(header)
                .append("--> Turn velocity gain : ")
                .append(mTurnVelocityGain)
                .append("\n");

        return result.toString();

    }

    /**
     * List of left motors  for tuning
     *
     * @return A list of the left motors
     */
    public List<MotorComponent>         left(){
        List<MotorComponent> result = new ArrayList<>();
        if(mConfigurationValid) {
            result = mLeft;
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
            result = mRight;
        }
        return result;
    }
}
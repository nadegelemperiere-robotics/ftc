/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Localization using 4 wheels encoders
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.odometers;

/* System includes */
import java.util.Map;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/* ACME robotics includes */
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.imus.ImuComponent;
import org.firstinspires.ftc.core.components.motors.MotorComponent;

public class DriveEncodersOdometer implements OdometerComponent {

    static  final String    sLeftFrontHwMapKey   = "front-left-wheel";
    static  final String    sLeftBackHwMapKey    = "back-left-wheel";
    static  final String    sRightFrontHwMapKey  = "front-right-wheel";
    static  final String    sRightBackHwMapKey   = "back-right-wheel";
    static  final String    sImuHwMapKey         = "imu";
    static  final String    sTrackWidthTicksKey  = "track-width-ticks";
    static  final String    sLateralInPerTickKey = "lateral-in-per-tick";
    static  final String    sInPerTickKey        = "in-per-tick";

    final LogManager                    mLogger;

    boolean                             mConfigurationValid;
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
    Encoder                             mLeftFront;
    Encoder                             mRightFront;
    Encoder                             mLeftBack;
    Encoder                             mRightBack;
    ImuComponent                        mImu;

    Pose2d                              mInitialPose;
    Pose2d                              mCurrentPose;
    PoseVelocity2d                      mCurrentVelocity;

    double                              mInPerTick;
    double                              mTrackWidthTicks;
    double                              mLateralInPerTick;

    double                              mLastHeading;
    double                              mLastLeftFrontPos;
    double                              mLastLeftBackPos;
    double                              mLastRightFrontPos;
    double                              mLastRightBackPos;

    MecanumKinematics                   mKinematics;

    final ElapsedTime                   mTimer;

    public  DriveEncodersOdometer(String name, HardwareMap hwMap, Map<String, MotorComponent> motors, Map<String,ImuComponent> imus,LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;
        mIsFirstTime        = true;

        mName               = name;
        mLeftFrontHwName    = "";
        mLeftBackHwName     = "";
        mRightFrontHwName   = "";
        mRightBackHwName    = "";
        mImuHwName          = "";

        mCurrentPose     = new Pose2d(new Vector2d(0,0),0);
        mInitialPose     = new Pose2d(new Vector2d(0,0),0);
        mCurrentVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);

        mMap        = hwMap;
        mMotors     = motors;
        mImus       = imus;
        mLeftFront  = null;
        mLeftBack   = null;
        mRightFront = null;
        mRightBack  = null;
        mImu        = null;

        mTimer = new ElapsedTime();

    }

    @Override
    public void         pose(Pose2d current) {
        if(mConfigurationValid) {
            mIsFirstTime = true;
            mInitialPose = current;
        }
    }

    @Override
    public void         update() {
        if (mConfigurationValid) {

            Twist2dDual<Time> twist;

            PositionVelocityPair leftFrontPosVel = mLeftFront.update();
            PositionVelocityPair leftBackPosVel = mLeftBack.update();
            PositionVelocityPair rightBackPosVel = mRightBack.update();
            PositionVelocityPair rightFrontPosVel = mRightFront.update();

            double heading = mImu.heading();

            if (!mIsFirstTime) {
                mIsFirstTime = true;

                twist = new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }
            else {
                double headingDelta = heading - mLastHeading;
                twist = mKinematics.forward(new MecanumKinematics.WheelIncrements<>(
                        new DualNum<Time>(new double[]{
                                (leftFrontPosVel.position - mLastLeftFrontPos),
                                leftFrontPosVel.velocity,
                        }).times(mInPerTick),
                        new DualNum<Time>(new double[]{
                                (leftBackPosVel.position - mLastLeftBackPos),
                                leftBackPosVel.velocity,
                        }).times(mInPerTick),
                        new DualNum<Time>(new double[]{
                                (rightBackPosVel.position - mLastRightBackPos),
                                rightBackPosVel.velocity,
                        }).times(mInPerTick),
                        new DualNum<Time>(new double[]{
                                (rightFrontPosVel.position - mLastRightFrontPos),
                                rightFrontPosVel.velocity,
                        }).times(mInPerTick)
                ));

                twist = new Twist2dDual<>(
                        twist.line,
                        DualNum.cons(headingDelta, twist.angle.drop(1))
                );
            }


            mLastLeftFrontPos = leftFrontPosVel.position;
            mLastLeftBackPos = leftBackPosVel.position;
            mLastRightBackPos = rightBackPosVel.position;
            mLastRightFrontPos = rightFrontPosVel.position;
            mLastHeading = heading;

            mTimer.reset();

            mCurrentPose = mCurrentPose.plus(twist.value());
            mCurrentVelocity = twist.velocity().value();

        }
    }

    @Override
    public Pose2d           pose() {
        return mCurrentPose;
    }

    @Override
    public PoseVelocity2d   velocity() { return mCurrentVelocity; }

    @Override
    public void             log() {
        if (mConfigurationValid) {

            mLogger.metric("x", mCurrentPose.position.x + " inches");
            mLogger.metric("y", mCurrentPose.position.y + " inches");
            mLogger.metric("heading", mCurrentPose.heading.toDouble() + " rad");

            mLogger.metric("vx",mCurrentVelocity.linearVel.x + " inches/s");
            mLogger.metric("vy",mCurrentVelocity.linearVel.y + " inches/s");
            mLogger.metric("vheading",mCurrentVelocity.angVel + " rad/s");
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
                    mLeftBack = new Encoder(mMotors.get(mLeftBackHwName), mLogger);
                }
            }
            if(reader.has(sLeftFrontHwMapKey)) {
                mLeftFrontHwName = reader.getString(sLeftFrontHwMapKey);
                if(mMotors.containsKey(mLeftFrontHwName)) {
                    mLeftFront = new Encoder(mMotors.get(mLeftFrontHwName), mLogger);
                }
            }
            if(reader.has(sRightBackHwMapKey)) {
                mRightBackHwName = reader.getString(sRightBackHwMapKey);
                if(mMotors.containsKey(mRightBackHwName)) {
                    mRightBack = new Encoder(mMotors.get(mRightBackHwName), mLogger);
                }
            }
            if(reader.has(sRightFrontHwMapKey)) {
                mRightFrontHwName = reader.getString(sRightFrontHwMapKey);
                if(mMotors.containsKey(mRightFrontHwName)) {
                    mRightFront = new Encoder(mMotors.get(mRightFrontHwName), mLogger);
                }
            }
            if(reader.has(sImuHwMapKey)) {
                mImuHwName = reader.getString(sImuHwMapKey);
                if(mImus.containsKey(mImuHwName)) {
                    mImu = mImus.get(mImuHwName);
                }
            }

            mInPerTick  = 1.0;
            mLateralInPerTick  = 1.0;
            mTrackWidthTicks = 1.0;

            if (reader.has(sInPerTickKey)) {
                mInPerTick = reader.getDouble(sInPerTickKey);
            }
            if (reader.has(sTrackWidthTicksKey)) {
                mTrackWidthTicks = reader.getDouble(sTrackWidthTicksKey);
            }
            if (reader.has(sLateralInPerTickKey)) {
                mLateralInPerTick = reader.getDouble(sLateralInPerTickKey);
            }

            mKinematics = new MecanumKinematics(
                    mInPerTick * mTrackWidthTicks, mInPerTick / mLateralInPerTick);

            Pose2d origin = new Pose2d(new Vector2d(0,0),0);

            this.pose(origin);

        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

        if(mLeftFront == null || !mLeftFront.isConfigured())    { mConfigurationValid = false; }
        if(mLeftBack == null || !mLeftBack.isConfigured())      { mConfigurationValid = false; }
        if(mRightFront == null || !mRightFront.isConfigured())  { mConfigurationValid = false; }
        if(mRightBack == null || !mRightBack.isConfigured())    { mConfigurationValid = false; }
        if(mImu == null)                                        { mConfigurationValid = false; }

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
                writer.put(sInPerTickKey,mInPerTick);
                writer.put(sTrackWidthTicksKey,mTrackWidthTicks);
                writer.put(sLateralInPerTickKey,mLateralInPerTick);

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
                    .append(" - IPT : ")
                    .append(mInPerTick)
                    .append(" - LIPT : ")
                    .append(mLateralInPerTick)
                    .append(" - TWT : ")
                    .append(mTrackWidthTicks)
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
                    .append(" - IPT : ")
                    .append(mInPerTick)
                    .append(" - LIPT : ")
                    .append(mLateralInPerTick)
                    .append(" - TWT : ")
                    .append(mTrackWidthTicks)
                    .append("\n");
        }

        return result.toString();

    }

}

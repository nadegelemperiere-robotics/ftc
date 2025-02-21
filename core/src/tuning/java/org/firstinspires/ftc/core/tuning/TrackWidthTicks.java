/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   InPerTick tuning tool - common to many odometers tuning
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.tuning;

/* System includes */
import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.ArrayList;

/* Android includes */
import android.os.Environment;

/* JSON includes */
import org.json.JSONObject;
import org.json.JSONArray;
import org.json.JSONException;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.VoltageSensor;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;
import org.firstinspires.ftc.core.tools.MidPointTimer;

/* Components includes */
import org.firstinspires.ftc.core.components.odometers.Encoder;
import org.firstinspires.ftc.core.components.imus.ImuComponent;
import org.firstinspires.ftc.core.components.motors.MotorComponent;

/* Subsystems includes */
import org.firstinspires.ftc.core.subsystems.DriveTrain;
import org.firstinspires.ftc.core.subsystems.MecanumDrive;
import org.firstinspires.ftc.core.subsystems.TankDrive;

public class TrackWidthTicks {

    /* ---------------- Members ---------------- */

    private final LogManager            mLogger;

    private       List<Encoder>         mEncoders;

    private final DriveTrain            mDriveTrain;
    private final VoltageSensor         mVoltageSensor;
    private final ImuComponent          mImu;
    private final List<MotorComponent>  mLeft;
    private final List<MotorComponent>  mRight;

    private  JSONArray                  mVoltageTimes;
    private  JSONArray                  mVoltageValues;
    private  JSONArray                  mHeadingVelocityTimes;
    private  JSONArray                  mHeadingVelocityValues;
    private  JSONArray                  mRollVelocityTimes;
    private  JSONArray                  mRollVelocityValues;
    private  JSONArray                  mPitchVelocityTimes;
    private  JSONArray                  mPitchVelocityValues;
    private  List<JSONArray>            mLeftPowerTimes;
    private  List<JSONArray>            mLeftPowerValues;
    private  List<JSONArray>            mRightPowerTimes;
    private  List<JSONArray>            mRightPowerValues;
    private  List<JSONArray>            mLeftVelocityTimes;
    private  List<JSONArray>            mLeftVelocityValues;
    private  List<JSONArray>            mRightVelocityTimes;
    private  List<JSONArray>            mRightVelocityValues;
    private  List<JSONArray>            mLeftPositionTimes;
    private  List<JSONArray>            mLeftPositionValues;
    private  List<JSONArray>            mRightPositionTimes;
    private  List<JSONArray>            mRightPositionValues;


    private double                      mPowerPerSecond;
    private double                      mMaxPower;
    private final MidPointTimer         mTimer;

    public TrackWidthTicks(DriveTrain train, ImuComponent imu, VoltageSensor sensor, LogManager logger)
    {
        mLogger         = logger;

        mDriveTrain     = train;
        mVoltageSensor  = sensor;
        mImu            = imu;
        mLeft           = mDriveTrain.left();
        mRight          = mDriveTrain.right();

        mPowerPerSecond = 0.5;
        mMaxPower       = 1;

        mTimer          = new MidPointTimer();

        mVoltageTimes           = new JSONArray();
        mVoltageValues          = new JSONArray();
        mHeadingVelocityTimes   = new JSONArray();
        mHeadingVelocityValues  = new JSONArray();
        mRollVelocityTimes      = new JSONArray();
        mRollVelocityValues     = new JSONArray();
        mPitchVelocityTimes     = new JSONArray();
        mPitchVelocityValues    = new JSONArray();

        mLeftPowerTimes         = new ArrayList<>();
        mLeftPowerValues        = new ArrayList<>();
        mLeftVelocityTimes      = new ArrayList<>();
        mLeftVelocityValues     = new ArrayList<>();
        mLeftPositionTimes      = new ArrayList<>();
        mLeftPositionValues     = new ArrayList<>();
        for (int i_left = 0; i_left < mLeft.size(); i_left ++) {
            mLeftPowerTimes.add(new JSONArray());
            mLeftPowerValues.add(new JSONArray());
            mLeftVelocityTimes.add(new JSONArray());
            mLeftVelocityValues.add(new JSONArray());
            mLeftPositionTimes.add(new JSONArray());
            mLeftPositionValues.add(new JSONArray());
        }

        mRightPowerTimes         = new ArrayList<>();
        mRightPowerValues        = new ArrayList<>();
        mRightVelocityTimes      = new ArrayList<>();
        mRightVelocityValues     = new ArrayList<>();
        mRightPositionTimes      = new ArrayList<>();
        mRightPositionValues     = new ArrayList<>();
        for (int i_right = 0; i_right < mRight.size(); i_right ++) {
            mRightPowerTimes.add(new JSONArray());
            mRightPowerValues.add(new JSONArray());
            mRightVelocityTimes.add(new JSONArray());
            mRightVelocityValues.add(new JSONArray());
            mRightPositionTimes.add(new JSONArray());
            mRightPositionValues.add(new JSONArray());
        }


    }

    public void power(double slope, double max) {
        mPowerPerSecond = slope;
        mMaxPower       = max;
    }

    public void start() {
        mTimer.reset();

        mVoltageTimes           = new JSONArray();
        mVoltageValues          = new JSONArray();
        mHeadingVelocityTimes   = new JSONArray();
        mHeadingVelocityValues  = new JSONArray();
        mRollVelocityTimes      = new JSONArray();
        mRollVelocityValues     = new JSONArray();
        mPitchVelocityTimes     = new JSONArray();
        mPitchVelocityValues    = new JSONArray();

        mLeftPowerTimes         = new ArrayList<>();
        mLeftPowerValues        = new ArrayList<>();
        mLeftVelocityTimes      = new ArrayList<>();
        mLeftVelocityValues     = new ArrayList<>();
        mLeftPositionTimes      = new ArrayList<>();
        mLeftPositionValues     = new ArrayList<>();
        for (int i_left = 0; i_left < mLeft.size(); i_left ++) {
            mLeftPowerTimes.add(new JSONArray());
            mLeftPowerValues.add(new JSONArray());
            mLeftVelocityTimes.add(new JSONArray());
            mLeftVelocityValues.add(new JSONArray());
            mLeftPositionTimes.add(new JSONArray());
            mLeftPositionValues.add(new JSONArray());
        }

        mRightPowerTimes         = new ArrayList<>();
        mRightPowerValues        = new ArrayList<>();
        mRightVelocityTimes      = new ArrayList<>();
        mRightVelocityValues     = new ArrayList<>();
        mRightPositionTimes      = new ArrayList<>();
        mRightPositionValues     = new ArrayList<>();
        for (int i_right = 0; i_right < mRight.size(); i_right ++) {
            mRightPowerTimes.add(new JSONArray());
            mRightPowerValues.add(new JSONArray());
            mRightVelocityTimes.add(new JSONArray());
            mRightVelocityValues.add(new JSONArray());
            mRightPositionTimes.add(new JSONArray());
            mRightPositionValues.add(new JSONArray());
        }

    }

    public int data() {
        return mVoltageValues.length();
    }

    public double power() {
        return Math.min(mTimer.seconds() * mPowerPerSecond, mMaxPower);
    }
    
    public void update() {


        double power = Math.min(mTimer.seconds() * mPowerPerSecond, mMaxPower);
        mDriveTrain.drive(0,0,power);

        try{

            for(int i_left = 0; i_left < mLeft.size(); i_left ++) {
                mLeftPowerValues.get(i_left).put(mLeft.get(i_left).power());
                mLeftPowerTimes.get(i_left).put(mTimer.split());
                mLeftVelocityValues.get(i_left).put(mLeft.get(i_left).velocity());
                mLeftVelocityTimes.get(i_left).put(mTimer.split());
                mLeftPositionValues.get(i_left).put(mLeft.get(i_left).currentPosition());
                mLeftPositionTimes.get(i_left).put(mTimer.split());
            }

            for(int i_right = 0; i_right < mRight.size(); i_right ++) {
                mRightPowerValues.get(i_right).put(mRight.get(i_right).power());
                mRightPowerTimes.get(i_right).put(mTimer.split());
                mRightVelocityValues.get(i_right).put(mRight.get(i_right).velocity());
                mRightVelocityTimes.get(i_right).put(mTimer.split());
                mRightPositionValues.get(i_right).put(mRight.get(i_right).currentPosition());
                mRightPositionTimes.get(i_right).put(mTimer.split());
            }

            mVoltageValues.put(mVoltageSensor.getVoltage());
            mVoltageTimes.put(mTimer.split());

            mHeadingVelocityValues.put(mImu.headingVelocity());
            mHeadingVelocityTimes.put(mTimer.split());
            mRollVelocityValues.put(0);
            mRollVelocityTimes.put(mTimer.split());
            mPitchVelocityValues.put(0);
            mPitchVelocityTimes.put(mTimer.split());

        }
        catch(JSONException e) {
            mLogger.error("Failed to add new data to json structure");
        }

    }

    public void stop() {
        mDriveTrain.drive(0,0,0);

    }

    public void process() {

        JSONObject all = new JSONObject();

        try{

            if(mDriveTrain instanceof MecanumDrive) { all.put("type","mecanum");}
            else if(mDriveTrain instanceof TankDrive) { all.put("type","tank");}

            JSONObject voltage = new JSONObject();
            voltage.put("values",mVoltageValues);
            voltage.put("times",mVoltageTimes);
            all.put("voltages",voltage);

            JSONArray  angularVelocity = new JSONArray();
            JSONObject roll = new JSONObject();
            roll.put("values",mRollVelocityValues);
            roll.put("times",mRollVelocityTimes);
            angularVelocity.put(roll);
            JSONObject pitch = new JSONObject();
            pitch.put("values",mPitchVelocityValues);
            pitch.put("times",mPitchVelocityTimes);
            angularVelocity.put(pitch);
            JSONObject heading = new JSONObject();
            heading.put("values",mHeadingVelocityValues);
            heading.put("times",mHeadingVelocityTimes);
            angularVelocity.put(heading);
            all.put("angVels",angularVelocity);

            JSONArray leftencpos = new JSONArray();
            for(int i_left = 0; i_left < mLeftPositionValues.size();i_left ++) {
                JSONObject left = new JSONObject();
                left.put("values",mLeftPositionValues.get(i_left));
                left.put("times",mLeftPositionTimes.get(i_left));
                leftencpos.put(left);
            }
            all.put("leftEncPositions",leftencpos);

            JSONArray leftencvel = new JSONArray();
            for(int i_left = 0; i_left < mLeftVelocityValues.size();i_left ++) {
                JSONObject left = new JSONObject();
                left.put("values",mLeftVelocityValues.get(i_left));
                left.put("times",mLeftVelocityTimes.get(i_left));
                leftencvel.put(left);
            }
            all.put("leftEncVels",leftencvel);

            JSONArray leftpwr = new JSONArray();
            for(int i_left = 0; i_left < mLeftPowerValues.size();i_left ++) {
                JSONObject left = new JSONObject();
                left.put("values",mLeftPowerValues.get(i_left));
                left.put("times",mLeftPowerTimes.get(i_left));
                leftpwr.put(left);
            }
            all.put("leftPowers",leftpwr);

            JSONArray rightencpos = new JSONArray();
            for(int i_right = 0; i_right < mRightPositionValues.size();i_right ++) {
                JSONObject right = new JSONObject();
                right.put("values",mRightPositionValues.get(i_right));
                right.put("times",mRightPositionTimes.get(i_right));
                rightencpos.put(right);
            }
            all.put("rightEncPositions",rightencpos);

            JSONArray rightencvel = new JSONArray();
            for(int i_right = 0; i_right < mRightVelocityValues.size();i_right ++) {
                JSONObject right = new JSONObject();
                right.put("values",mRightVelocityValues.get(i_right));
                right.put("times",mRightVelocityTimes.get(i_right));
                rightencvel.put(right);
            }
            all.put("rightEncVels",rightencvel);

            JSONArray rightpwr = new JSONArray();
            for(int i_right = 0; i_right < mRightPowerValues.size();i_right ++) {
                JSONObject right = new JSONObject();
                right.put("values",mRightPowerValues.get(i_right));
                right.put("times",mRightPowerTimes.get(i_right));
                rightpwr.put(right);
            }
            all.put("rightPowers",rightpwr);



        }
        catch(JSONException e) {
            mLogger.error("Failed to create json to store experiment data");
        }

        String filename = Environment.getExternalStorageDirectory().getPath() + "/RoadRunner/tuning/angular-ramp/" +
                System.currentTimeMillis() + ".json";
        try {
            BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filename), StandardCharsets.UTF_8));
            writer.write(all.toString(4)); // 4 is the indentation level for pretty printing
            writer.close();
        }
        catch(JSONException | IOException e) { mLogger.error("Unable to write data json file " + filename); }

    }


}
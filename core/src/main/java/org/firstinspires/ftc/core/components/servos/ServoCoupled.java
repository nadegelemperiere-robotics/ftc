/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   ServoCoupled class supersedes the FTC servo class to manage
   A couple of servos both turning the same hardware.

   Note that this is a dangerous situation which can result in
   servo destruction if not correctly tuned. The coupled servos
   shall be tuned so that each orientation of the hardware they
   both support correspond to the same position on the 2 servos.
   If wrongly tuned, each of the 2 coupled servos may end up
   each forcing into a position they can not reach without the
   other failing.

   This means for example that the 2 servos are the same model
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.servos;


/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class ServoCoupled implements ServoComponent {

    public static final String  sFirstKey  = "first";
    public static final String  sSecondKey = "second";

    LogManager                  mLogger;

    String                      mName;
    String                      mFirstHwName;
    String                      mSecondHwName;

    boolean                     mConfigurationValid;

    ServoControllerComponent    mController;

    Servo.Direction             mDirection;

    HardwareMap                 mMap;
    Servo                       mFirst;
    Servo                       mSecond;

    /* -------------- Constructors --------------- */
    public ServoCoupled(String name, HardwareMap hwMap, LogManager logger)
    {
        mLogger                 = logger;

        mName                   = name;
        mFirstHwName            = "";
        mSecondHwName           = "";
        mConfigurationValid     = false;

        mMap                    = hwMap;

        mFirst                  = null;
        mSecond                 = null;
        mController             = null;
        mDirection              = Servo.Direction.FORWARD;
    }

    /* --------------------- Custom functions ---------------------- */

    @Override
    public String                       getName() { return mName; }

    /* ------------------ Configurable functions ------------------- */

    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    @Override
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        mFirst = null;
        mSecond = null;

        if(!reader.has(sFirstKey))       { mLogger.error("Missing first Servo for coupled servo"); }
        else if(!reader.has(sSecondKey)) { mLogger.error("Missing second Servo for coupled servo"); }
        else {
            try {
                JSONObject first = reader.getJSONObject(sFirstKey);
                JSONObject second = reader.getJSONObject(sSecondKey);

                if(first.has(sHwMapKey)) {
                    mFirstHwName = first.getString(sHwMapKey);
                    mFirst = mMap.tryGet(Servo.class, mFirstHwName);
                }
                if(second.has(sHwMapKey)) {
                    mSecondHwName = second.getString(sHwMapKey);
                    mSecond = mMap.tryGet(Servo.class, mSecondHwName);
                }

                if (mFirst != null && first.has(sReverseKey)) {
                    boolean shallReverse = first.getBoolean(sReverseKey);
                    if(shallReverse) { mFirst.setDirection(Servo.Direction.REVERSE); }
                    else { mFirst.setDirection(Servo.Direction.FORWARD); }
                }
                else if(mFirst != null) { mFirst.setDirection(Servo.Direction.FORWARD);  }

                if (mSecond != null && second.has(sReverseKey)) {
                    boolean shallReverse = second.getBoolean(sReverseKey);
                    if(shallReverse) { mSecond.setDirection(Servo.Direction.REVERSE); }
                    else { mSecond.setDirection(Servo.Direction.FORWARD); }
                }
                else if(mSecond != null) { mSecond.setDirection(Servo.Direction.FORWARD);  }

                if(mFirst != null && mSecond != null ) {
                    mController             = new ServoControllerCoupled(mFirst.getController(), mSecond.getController(), mName, mLogger);
                }


            }
            catch(JSONException e) { mLogger.error(e.getMessage()); }
        }

        if (mFirst == null) { mConfigurationValid = false; }
        if (mSecond == null) { mConfigurationValid = false; }

    }

    @Override
    public void                         write(JSONObject writer) {

        JSONObject first = new JSONObject();
        JSONObject second = new JSONObject();

        try {
            if(mFirst != null) {
                first.put(sHwMapKey,mFirstHwName);
                first.put(sReverseKey,mFirst.getDirection() == Servo.Direction.REVERSE);
            }
            if(mSecond != null) {
                second.put(sHwMapKey,mSecondHwName);
                second.put(sReverseKey,mSecond.getDirection() == Servo.Direction.REVERSE);
            }

            writer.put(sFirstKey,first);
            writer.put(sSecondKey,second);
        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

    }

    @Override
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        if (mFirst != null) {
            result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                    .append("ID : ")
                    .append(sFirstKey)
                    .append(" - HW : ")
                    .append(mFirstHwName)
                    .append(" - REV : ")
                    .append(mFirst.getDirection() == Servo.Direction.REVERSE)
                    .append("</li>\n");
        }
        if (mSecond != null) {
            result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                    .append("ID : ")
                    .append(sSecondKey)
                    .append(" - HW : ")
                    .append(mSecondHwName)
                    .append(" - REV : ")
                    .append(mSecond.getDirection() == Servo.Direction.REVERSE)
                    .append("</li>\n");
        }

        return result.toString();

    }
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();

        if (mFirst != null) {
            result.append(header)
                    .append("> ")
                    .append(sFirstKey)
                    .append(" HW :")
                    .append(mSecondHwName)
                    .append(" - REV : ")
                    .append(mFirst.getDirection() == Servo.Direction.REVERSE)
                    .append("\n");
        }
        if (mSecond != null) {
            result.append(header)
                    .append("> ")
                    .append(sSecondKey)
                    .append(" HW : ")
                    .append(mSecondHwName)
                    .append(" - REV : ")
                    .append(mSecond.getDirection() == Servo.Direction.REVERSE)
                    .append("\n");
        }

        return result.toString();

    }


    /* ---------------------- Servo functions ---------------------- */

    @Override
    public ServoControllerComponent     getController() {
        return mController;
    }

    @Override
    public Servo.Direction	            getDirection()
    {
        return mDirection;
    }

    @Override
    public double	                    getPosition()
    {
        double result = -1;
        if(mConfigurationValid) {
            result = 0.5 * mFirst.getPosition() + 0.5 * mSecond.getPosition();
        }
        return result;
    }

    @Override
    public void	                        scaleRange(double min, double max)
    {
        if(mConfigurationValid) {
            mFirst.scaleRange(min, max);
            mSecond.scaleRange(min, max);
        }
    }

    @Override
    public void	                        setDirection(Servo.Direction direction)
    {
        if(direction != mDirection && mConfigurationValid) {

            if(     mFirst.getDirection()  == Servo.Direction.FORWARD) { mFirst.setDirection(Servo.Direction.REVERSE);  }
            else if(mFirst.getDirection()  == Servo.Direction.REVERSE) { mFirst.setDirection(Servo.Direction.FORWARD);  }

            if(     mSecond.getDirection() == Servo.Direction.FORWARD) { mSecond.setDirection(Servo.Direction.REVERSE); }
            else if(mSecond.getDirection() == Servo.Direction.REVERSE) { mSecond.setDirection(Servo.Direction.FORWARD); }

            mDirection = direction;

        }
    }

    @Override
    public void	                        setPosition(double position)
    {
        if(mConfigurationValid) {
            mFirst.setPosition(position);
            mSecond.setPosition(position);
        }
    }
}

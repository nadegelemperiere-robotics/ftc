/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   A single servo
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.servos;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;
import org.json.JSONException;
import org.json.JSONObject;

public class ServoSingle implements ServoComponent {

    LogManager                  mLogger;

    boolean                     mConfigurationValid;
    String                      mName;
    String                      mHwName;

    ServoControllerComponent    mController;

    Servo.Direction             mDirection;

    HardwareMap                 mMap;
    Servo                       mServo;

    /* -------------- Constructors --------------- */
    public ServoSingle(String name, HardwareMap hwMap, LogManager logger)
    {
        mConfigurationValid = false;

        mLogger             = logger;

        mName               = name;
        mHwName             = "";

        mMap                = hwMap;

        mController         = null;
        mServo              = null;
        mDirection          = Servo.Direction.FORWARD;

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
        mServo = null;

        try {
            if(reader.has(sHwMapKey)) {
                mHwName = reader.getString(sHwMapKey);
                mServo = mMap.tryGet(Servo.class,mHwName);
            }

            if (mServo != null && reader.has(sReverseKey)) {
                boolean shallReverse = reader.getBoolean(sReverseKey);
                if(shallReverse) { mServo.setDirection(Servo.Direction.REVERSE); }
                else { mServo.setDirection(Servo.Direction.FORWARD); }
            }

            if(mServo != null) {
                mController             = new ServoControllerSingle(mServo.getController(), mName, mLogger);
            }

        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

        if (mServo == null) { mConfigurationValid = false; }

    }

    @Override
    public void                         write(JSONObject writer) {

        try {
            writer.put(sHwMapKey,mHwName);
            writer.put(sReverseKey,mServo.getDirection() == Servo.Direction.REVERSE);

        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

    }

    @Override
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        if (mServo != null) {
            result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                    .append("HW : ")
                    .append(mHwName)
                    .append(" - REV : ")
                    .append(mServo.getDirection() == Servo.Direction.REVERSE)
                    .append("</li>\n");
        }

        return result.toString();

    }
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();

        if (mServo != null) {
            result.append(header)
                    .append("> HW :")
                    .append(mHwName)
                    .append(" - REV : ")
                    .append(mServo.getDirection() == Servo.Direction.REVERSE)
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
        Servo.Direction result = Servo.Direction.FORWARD;
        if(mConfigurationValid) {
            result = mServo.getDirection();
        }
        return result;
    }

    @Override
    public double	                    getPosition()
    {
        double result = -1;
        if(mConfigurationValid) {
            result = mServo.getPosition();
        }
        return result;
    }

    @Override
    public void	                        scaleRange(double min, double max)
    {
        if(mConfigurationValid) {
            mServo.scaleRange(min, max);
        }
    }

    @Override
    public void	                        setDirection(Servo.Direction direction)
    {
        if(mConfigurationValid) {
            mServo.setDirection(direction);
        }
    }

    @Override
    public void	                        setPosition(double position)
    {
        if(mConfigurationValid) {
            mServo.setPosition(position);
        }
    }
}

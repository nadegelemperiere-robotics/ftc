/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   A single servo
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

    /**
     * Returns the servo reference name.
     *
     * @return the servo name
     */
    @Override
    public String                       name() { return mName; }


    /* ------------------ Configurable functions ------------------- */
    /**
     * Determines if the coupled servo component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /**
     * Reads and applies the servo configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    @Override
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        mServo = null;

        try {
            if(mMap != null && reader.has(sHwMapKey)) {
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

    /**
     * Writes the current servo configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) {

        if(mConfigurationValid) {

            try {
                writer.put(sHwMapKey, mHwName);
                writer.put(sReverseKey, mServo.getDirection() == Servo.Direction.REVERSE);

            } catch (JSONException e) { mLogger.error(e.getMessage()); }
        }

    }

    /**
     * Generates an HTML representation of the servo configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted servo configuration.
     */
    @Override
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {

            if (mServo != null) {
                result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                        .append("HW : ")
                        .append(mHwName)
                        .append(" - REV : ")
                        .append(mServo.getDirection() == Servo.Direction.REVERSE)
                        .append("</li>\n");
            }
        }
        return result.toString();

    }

    /**
     * Generates a text-based representation of the servo configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted servo configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();
        if(mConfigurationValid) {

            if (mServo != null) {
                result.append(header)
                        .append("> HW :")
                        .append(mHwName)
                        .append(" - REV : ")
                        .append(mServo.getDirection() == Servo.Direction.REVERSE)
                        .append("\n");
            }
        }

        return result.toString();

    }

    /* ---------------------- Servo functions ---------------------- */

    /**
     * Retrieves the servo controller managing this component.
     *
     * @return The associated ServoControllerComponent.
     */
    @Override
    public ServoControllerComponent     controller() {
        return mController;
    }

    /**
     * Retrieves the current direction of the servo.
     *
     * @return The direction of the servo (FORWARD or REVERSE).
     */
    @Override
    public Servo.Direction	            direction()
    {
        Servo.Direction result = Servo.Direction.FORWARD;
        if(mConfigurationValid) {
            result = mServo.getDirection();
        }
        return result;
    }

    /**
     * Retrieves the position of the servo.
     *
     * @return The servo position in the range [0,1], or -1 if not configured.
     */
    @Override
    public double	                    position()
    {
        double result = -1;
        if(mConfigurationValid) {
            result = mServo.getPosition();
        }
        return result;
    }

    /**
     * Scales the range of motion for the servos.
     *
     * @param min The new minimum position (0.0 to 1.0).
     * @param max The new maximum position (0.0 to 1.0).
     */
    @Override
    public void	                        scaleRange(double min, double max)
    {
        if(mConfigurationValid) {
            mServo.scaleRange(min, max);
        }
    }

    /**
     * Sets the direction of the servos.
     *
     * @param direction The new direction (FORWARD or REVERSE).
     */
    @Override
    public void	                        direction(Servo.Direction direction)
    {
        if(mConfigurationValid) {
            mServo.setDirection(direction);
        }
    }

    /**
     * Sets the position of the servos.
     *
     * @param position The new position to reach
     */
    @Override
    public void	                        position(double position)
    {
        if(mConfigurationValid) {
            mServo.setPosition(position);
        }
    }
}

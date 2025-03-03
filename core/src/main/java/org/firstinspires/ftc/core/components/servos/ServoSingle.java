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

    final LogManager            mLogger;

    boolean                     mConfigurationValid;
    final String                mName;
    String                      mHwName;

    ServoControllerComponent    mController;

    final HardwareMap           mMap;
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

    }
    /* --------------------- Custom functions ---------------------- */

    /**
     * Returns the servo reference name.
     *
     * @return the servo name
     */
    @Override
    public String                       getName() { return mName; }

    /**
     * Logs the current servo position.
     */
    @Override
    public void                         log() {
        if (mConfigurationValid) {
            mLogger.metric(LogManager.Target.DASHBOARD, mName+"-pos","" + mServo.getPosition());
        }
    }


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

    /* ------------------ HardwareDevice functions ----------------- */

    /**
     * Returns an indication of the manufacturer of this device.
     * @return the manufacturer
     */
    @Override
    public Manufacturer                 getManufacturer()
    {
        Manufacturer result = Manufacturer.Unknown;
        if(mConfigurationValid) {
            result = mServo.getManufacturer();
        }
        return result;
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.Note that this is a device-type-specific name; it has nothing to do with the name by which a user might have configured the device in a robot configuration.
     * @return the device name
     */
    @Override
    public String                       getDeviceName()
    {
        String result = "";
        if(mConfigurationValid) {
            result = mServo.getDeviceName();
        }
        return result;
    }

    /**
     * Get connection information about this device in a human readable format
     * @return connection information
     */
    @Override
    public String                       getConnectionInfo() {
        String result = "";
        if(mConfigurationValid) {
            result = mServo.getConnectionInfo();
        }
        return result;
    }

    /**
     * Version
     */
    @Override
    public int                          getVersion() {
        int result = -1;
        if(mConfigurationValid) {
            result = mServo.getVersion();
        }
        return result;
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void                         resetDeviceConfigurationForOpMode() {
        if(mConfigurationValid) {
            mServo.resetDeviceConfigurationForOpMode();
        }
    }

    /**
     * Closes this device
     */
    @Override
    public void                         close()
    {
        if(mConfigurationValid) {
            mServo.close();
        }
    }

    /* ---------------------- Servo functions ---------------------- */

    /**
     * Retrieves the servo controller managing this component.
     *
     * @return The associated ServoControllerComponent.
     */
    @Override
    public ServoControllerComponent     getController() {
        return mController;
    }

    /**
     * Unable to provide this method since each motor has a difference port
     * @return -1
     */
    @Override
    public int                          getPortNumber() {
        int result = -1;
        if(mConfigurationValid) {
            result = mServo.getPortNumber();
        }
        return result;
    }

    /**
     * Retrieves the current direction of the servo.
     *
     * @return The direction of the servo (FORWARD or REVERSE).
     */
    @Override
    public Servo.Direction	            getDirection()
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
    public double	                    getPosition()
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
    public void	                        setDirection(Servo.Direction direction)
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
    public void	                        setPosition(double position)
    {
        if(mConfigurationValid) {
            mServo.setPosition(position);
        }
    }
}

/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Actuator subsystem
   ------------------------------------------------------- */
package org.firstinspires.ftc.core.subsystems;

/* System includes */
import java.util.Iterator;
import java.util.Map;
import java.util.LinkedHashMap;

/* Json includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;
import org.firstinspires.ftc.core.tools.Timer;

/* Components includes */
import org.firstinspires.ftc.core.components.motors.MotorComponent;
import org.firstinspires.ftc.core.components.servos.ServoComponent;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

public class Actuator implements Subsystem {

    static  final public String sMotorKey           = "motor";
    static  final public String sServoKey           = "servo";
    static  final public String sPositionsKey       = "positions";
    static  final public String sPowersKey          = "powers";
    static  final public String sSetPositionKey     = "set-position";
    static  final public String sHoldPositionKey    = "hold-position";

    protected LogManager            mLogger;

    protected boolean               mConfigurationValid;
    boolean                         mHasFinished;

    String                          mName;
    String                          mHwName;

    protected Hardware              mHardware;
    protected MotorComponent        mMotor;
    protected ServoComponent        mServo;

    protected Map<String, Double>   mPositions;
    protected String                mPosition;
    protected Double                mOffset;

    Timer                           mTimer;

    Double                          mTolerance;
    Double                          mSetPositionPower;
    Double                          mHoldPositionPower;

    /**
     * Constructor
     *
     * @param name The subsystem name
     * @param hardware The robot current hardware
     * @param logger The logger to report events
     */
    public Actuator(String name, Hardware hardware, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;

        mName               = name;
        mHwName             = "";

        mHardware           = hardware;
        mMotor              = null;
        mServo              = null;

        mPositions          = new LinkedHashMap<>();
        mPosition           = "";

        mTimer              = new Timer(mLogger);

        mSetPositionPower   = 1.0;
        mHoldPositionPower  = 0.0;

    }

    /**
     * Check if the actuator has reached his position
     *
     * @return true if the position was reached, false otherwise
     */
    public boolean                      hasFinished() { return mHasFinished;}

    /**
     * Sets the current position offset from the reference position to the real robot state
     *
     * @param offset the offset in ticks
     */
    public  void                        offset( double offset ) { mOffset = offset; }

    /**
     * Returns the current position offset from the reference position to the real robot state
     *
     * @return the offset in ticks
     */
    public  double                      offset()                { return mOffset;   }

    /**
     * Returns the name of the current actuator position
     *
     * @return the name of the position, empty string if moving freely
     */
    public  String                      position()              { return mPosition; }


    /**
     * Update periodically actuator status
     */
    public void                         update() {
        if (mServo != null) { mHasFinished = !(mTimer.isArmed()); }
        if (mMotor != null) {
            if(!mHasFinished) {
                mHasFinished = !(mMotor.isBusy()) || !(mTimer.isArmed());
                if(mHasFinished) { mMotor.power(mHoldPositionPower); }
            }
        }
    }

    /**
     * Position the actuator on one of its reference positions
     *
     * @param position name of the position to reach
     * @param timeout time left to the actuator before abort
     */
    public void                         position(String position, int tolerance, int timeout) {

        if( mMotor != null && mPositions.containsKey(position) && mConfigurationValid && this.hasFinished()) {

            mMotor.targetPositionTolerance(tolerance);
            mTolerance = ((double)tolerance);

            Double target = mPositions.get(position);
            if(target != null) {
                mMotor.targetPosition((int)((double)target - mOffset));
                mMotor.mode(DcMotor.RunMode.RUN_TO_POSITION);
                mHasFinished = false;
                mMotor.power(mSetPositionPower);
                mPosition = position;
                mTimer.arm(timeout);
            }

        }
        if( mServo != null && mPositions.containsKey(position) && mConfigurationValid) {
            Double target = mPositions.get(position);
            if(target != null) { mServo.position(target); }
            mPosition = position;
            mHasFinished = false;
            mTimer.arm(timeout);
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
     * Reads and applies the actuator configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    public  void                        read(JSONObject reader) {

        mMotor = null;
        mServo = null;
        mPositions.clear();

        try {
            if(reader.has(sMotorKey)) {
                mHwName = reader.getString(sMotorKey);
                Map<String,MotorComponent> motors = mHardware.motors();
                if(motors.containsKey(mHwName)) { mMotor = motors.get(mHwName); }
            }
            if(reader.has(sServoKey)) {
                mHwName = reader.getString(sServoKey);
                Map<String,ServoComponent> servos = mHardware.servos();
                if(servos.containsKey(mHwName)) { mServo = servos.get(mHwName); }
            }

            if(mMotor != null && reader.has(sPowersKey)) {
                if(reader.has(sSetPositionKey)) {
                    mSetPositionPower = reader.getDouble(sSetPositionKey);
                }
                if(reader.has(sHoldPositionKey)) {
                    mHoldPositionPower = reader.getDouble(sHoldPositionKey);
                }
            }

            if(reader.has(sPositionsKey)) {
               JSONObject positions = reader.getJSONObject(sPositionsKey);
               Iterator<String> keys = positions.keys();
                while (keys.hasNext()) {
                    String key = keys.next();
                    mPositions.put(key, positions.getDouble(key));
                }
            }

        } catch(JSONException e) {
            mLogger.error(e.getMessage());
        }

        if(mMotor == null && mServo == null) {
            mLogger.error("No servo nor motor found for actuator " + mName);
            mConfigurationValid = false;
        }
        if(mMotor != null && mServo != null) {
            mLogger.error("Servo and motor found for actuator " + mName);
            mConfigurationValid = false;
        }
        if(mPositions.isEmpty()) {
            mLogger.warning("No reference positions for actuator " + mName);
        }

    }

    /**
     * Writes the current actuator configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    public void                         write(JSONObject writer) {

        if(mConfigurationValid) {

            try {
                if(mMotor != null) { writer.put(sMotorKey, mHwName); }
                if(mServo != null) { writer.put(sServoKey, mHwName); }

                JSONObject positions = new JSONObject();
                for (Map.Entry<String, Double> entry : mPositions.entrySet()) {
                    positions.put(entry.getKey(), entry.getValue());
                }
                writer.put(sPositionsKey, positions);

            } catch(JSONException e) {
                mLogger.error(e.getMessage());
            }

        }
    }

    /**
     * Generates an HTML representation of the actuator configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted actuator configuration.
     */
    @Override
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {

            if (mMotor != null) {
                result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                        .append(" - MOTOR ")
                        .append(mHwName)
                        .append("\n");
            }
            if (mServo != null) {
                result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                        .append(" - SERVO ")
                        .append(mHwName)
                        .append("\n");
            }

            if(!mPositions.isEmpty()) {
                result.append("<details>\n");
                result.append("<summary style=\"font-size: 12px; font-weight: 500\"> POSITIONS </summary>\n");
                result.append("<ul>\n");

                for (Map.Entry<String, Double> position : mPositions.entrySet()) {
                    result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                            .append(position.getKey())
                            .append(" : ")
                            .append(position.getValue())
                            .append("</li>");
                    }

                result.append("</ul>\n");
                result.append("</details>\n");
            }
        }

        return result.toString();

    }

    /**
     * Generates a text-based representation of the actuator configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted actuator configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {
            if (mMotor != null) {
                result.append(header)
                        .append("> NOTOR : ")
                        .append(mHwName)
                        .append("\n");
            }
            if (mServo != null) {
                result.append(header)
                        .append("> SERVO : ")
                        .append(mHwName)
                        .append("\n");
            }

            if(!mPositions.isEmpty()) {
                result.append(header)
                        .append("> POSITIONS\n");
                for (Map.Entry<String, Double> position : mPositions.entrySet()) {
                    result.append(header)
                            .append("-->")
                            .append(position.getKey())
                            .append(" : ")
                            .append(position.getValue())
                            .append("\n");
                }
            }
        }

        return result.toString();

    }

}
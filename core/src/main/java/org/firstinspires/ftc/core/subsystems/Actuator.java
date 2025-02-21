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

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.InterOpMode;

public class Actuator extends Subsystem {

    public static final String sMotorKey        = "motor";
    public static final String sServoKey        = "servo";
    public static final String sPositionsKey    = "positions";
    public static final String sPowersKey       = "powers";
    public static final String sSetPositionKey  = "set-position";
    public static final String sHoldPositionKey = "hold-position";
    public static final String sShortNameKey    = "short";

    protected final LogManager          mLogger;

    protected boolean                   mConfigurationValid;
    boolean                             mHasFinished;

    final String                        mName;
    String                              mShortName;
    String                              mHwName;

    protected final Hardware            mHardware;
    protected MotorComponent            mMotor;
    protected ServoComponent            mServo;

    protected final Map<String, Double> mPositions;
    protected String                    mPosition;

    final Timer                         mTimer;

    double                              mTolerance;
    protected double                    mOffset;
    double                              mSetPositionPower;
    double                              mHoldPositionPower;

    /**
     * Constructor
     *
     * @param name     The subsystem name
     * @param hardware The robot current hardware
     * @param logger   The logger to report events
     */
    public Actuator(String name, Hardware hardware, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;

        mName               = name;
        mHwName             = "";
        mShortName          = "";

        mHardware           = hardware;
        mMotor              = null;
        mServo              = null;

        mPositions          = new LinkedHashMap<>();
        mPosition           = "none";

        mTimer              = new Timer(mLogger);
        mHasFinished        = true;

        mSetPositionPower   = 1.0;
        mHoldPositionPower  = 0.0;
        mTolerance          = 0.0;

        // Reading offset from interopmodes stored data if exist
        mOffset             = 0.0;
        Object offset = InterOpMode.instance().get(mName + "-offset");
        if(offset != null) { mOffset = (double) offset; }


    }

    /**
     * Check if the actuator has reached his position
     *
     * @return true if the position was reached, false otherwise
     */
    public boolean hasFinished() {
        return mHasFinished;
    }

    /**
     * Returns the name of the current actuator position
     *
     * @return the name of the position, empty string if moving freely
     */
    public String position() {
        return mPosition;
    }

    /**
     * Update periodically actuator status
     */
    public void                         update() {
        mLogger.debug(LogManager.Target.FILE,mName + " start");
        if (mServo != null) {
            mHasFinished = !(mTimer.isArmed());
        }
        if (mMotor != null) {
            if (!mHasFinished) {
                mHasFinished = !(mMotor.isBusy()) || !(mTimer.isArmed());
                if(!mTimer.isArmed() && mMotor.isBusy()) { mLogger.warning(mName + " timeouted"); }
                if (mHasFinished) {
                    mMotor.power(mHoldPositionPower);
                    double error = Math.abs(mMotor.currentPosition() - mMotor.targetPosition());
                    mLogger.info(mName + " finished with error " + error + " for tolerance " + mMotor.targetPositionTolerance() );
                    mMotor.log();
                }
            }
        }
        mLogger.debug(LogManager.Target.FILE,mName + " stop");
    }

    /**
     * Log actuator current status
     */
    public void                         log() {

        if(mMotor != null && this.hasFinished()) {
            //mMotor.log();
            mLogger.info(mShortName + " : pos = " + mPosition + " - enc : " + mMotor.currentPosition() + " - spd : " + mMotor.velocity() + " - pwr : " + mMotor.power() + " - mode : " + mMotor.mode());
        }
        else if(mMotor != null && !this.hasFinished()) {
            //mMotor.log();
            mLogger.info(mShortName + " : pos > " + mPosition + " - enc : " + mMotor.currentPosition() + " - spd : " + mMotor.velocity() + " - pwr : " + mMotor.power() + " - mode : " + mMotor.mode());
        }
        else if(mServo != null && this.hasFinished()) {
            mLogger.info(mShortName + " : pos = " + mPosition + " - srv : " + mServo.position());
        }
        else if(mServo != null && !this.hasFinished()) {
            mLogger.info(mShortName + " : pos > " + mPosition + " - srv : " + mServo.position());
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
            mTolerance = tolerance;

            Double target = mPositions.get(position);
            if(target != null) {
                mMotor.targetPosition((int)(target - mOffset));
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
     * Persist data to be able to keep the same behavior after a reinitialization.
     * If the motor is stopped in a position that is not its 0, next OpMode, it will
     * Restart as if the current position was its 0. Therefore, all the registered
     * positions will be offseted. We store the current position to remember that
     * a motor 0 now corresponds to this position
     */
    @Override
    public void                         persist()
    {
        if(mMotor != null) {
            // Now the 0 position in actuator reference will correspond to the previous offset + the current position
            // Example :
            // - Stopping first at a motor position of 320
            // - Storing offset = 320 then reset motor to 0 -> Now accessing position 0 means asking motor for position -320
            // - Stopping a second time at motor position of -320
            // - Storing offset +320 -320 = 0 -> Accessing position 0 is now back to asking motor position 0
            InterOpMode.instance().add(mName + "-offset", mOffset + (double)mMotor.currentPosition());
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
     * Reads and applies the actuator configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    public  void                        read(JSONObject reader) {

        mMotor = null;
        mServo = null;
        mConfigurationValid = true;
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
                JSONObject powers = reader.getJSONObject(sPowersKey);
                if(powers.has(sSetPositionKey)) {
                    mSetPositionPower = powers.getDouble(sSetPositionKey);
                }
                if(powers.has(sHoldPositionKey)) {
                    mHoldPositionPower = powers.getDouble(sHoldPositionKey);
                }
            }

            if(reader.has(sShortNameKey)) {
                mShortName = reader.getString(sShortNameKey);
            }
            if(mShortName.isEmpty()) { mShortName = mName; }

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
                writer.put(sTypeKey,"actuator");
            } catch(JSONException e) {
                mLogger.error(e.getMessage());
            }

            writeWithoutType(writer);

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

            result.append("<li style=\"padding-left:10px; font-size: 10px\">")
                    .append(" <span style=\"font-weight: 500\"> SHORT : </span> ")
                    .append(mShortName)
                    .append("\n");

            if (mMotor != null) {
                result.append("<li style=\"padding-left:10px; font-size: 10px\">")
                        .append(" <span style=\"font-weight: 500\"> MOTOR : </span> ")
                        .append(mHwName)
                        .append("\n");
            }
            if (mServo != null) {
                result.append("<li style=\"padding-left:10px; font-size: 10px\">")
                        .append(" <span style=\"font-weight: 500\"> SERVO : </span> ")
                        .append(mHwName)
                        .append("\n");
            }

            if(!mPositions.isEmpty()) {
                result.append("<details>\n");
                result.append("<summary style=\"font-size: 10px; font-weight: 500\"> POSITIONS </summary>\n");
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
            result.append(header)
                    .append("> SHORT : ")
                    .append(mShortName)
                    .append("\n");

            if (mMotor != null) {
                result.append(header)
                        .append("> MOTOR : ")
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
                            .append("--> ")
                            .append(position.getKey())
                            .append(" : ")
                            .append(position.getValue())
                            .append("\n");
                }
            }
        }

        return result.toString();

    }

    /**
     * Writes the current actuator configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    protected void                      writeWithoutType(JSONObject writer) {

        if(mConfigurationValid) {

            try {

                writer.put(sShortNameKey,mShortName);

                if(mMotor != null) { writer.put(sMotorKey, mHwName); }
                if(mServo != null) { writer.put(sServoKey, mHwName); }

                JSONObject positions = new JSONObject();
                for (Map.Entry<String, Double> entry : mPositions.entrySet()) {
                    positions.put(entry.getKey(), entry.getValue());
                }
                writer.put(sPositionsKey, positions);

                if(mMotor != null) {
                    JSONObject powers = new JSONObject();
                    powers.put(sSetPositionKey, mSetPositionPower);
                    powers.put(sHoldPositionKey, mHoldPositionPower);
                    writer.put(sPowersKey, powers);
                }

            } catch(JSONException e) {
                mLogger.error(e.getMessage());
            }

        }
    }

}
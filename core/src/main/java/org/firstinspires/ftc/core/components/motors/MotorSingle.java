/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   CoupledMotor class overloads the FTC motor class to manage
   A couple of motors both turning the same hardware.

   Note that this is a dangerous situation which can result in
   motor destruction if not correctly tuned. The coupled motors
   shall be the same model
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.motors;


/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;
import org.json.JSONException;
import org.json.JSONObject;


public class MotorSingle implements MotorComponent {

    final LogManager    mLogger;

    boolean             mConfigurationValid;
    final String        mName;
    String              mHwName;

    final HardwareMap   mMap;
    DcMotorEx           mMotor;

    int                 mInvertPosition;

    /* ----------------------- Constructors ------------------------ */
    /**
     * Constructs a MotorSingle instance.
     *
     * @param name   The name of the single motor component.
     * @param hwMap  The FTC HardwareMap to retrieve motor hardware.
     * @param logger The logging manager for error reporting and debugging.
     */
    public MotorSingle(String name, HardwareMap hwMap, LogManager logger)
    {
        mLogger             = logger;
        mName               = name;
        mHwName             = "";
        mConfigurationValid = false;

        mMap                = hwMap;

        mMotor              = null;
        mInvertPosition     = 1;
        
    }

    /* --------------------- Custom functions ---------------------- */

    /**
     * Retrieves the name of the single motor component.
     *
     * @return The name of the component.
     */
    @Override
    public String                       name() { return mName; }

    /**
     * Determines if encoder correction is required.
     *
     * @return True if at least one motor has inverted encoder behavior, false otherwise.
     */
    @Override
    public boolean                      encoderCorrection() { return (mInvertPosition == -1);}

    /**
     * Enables or disables encoder correction.
     *
     * @param shallCorrect True to enable encoder correction, false to disable.
     */
    @Override
    public void                         encoderCorrection( boolean shallCorrect) {
        if (shallCorrect) { mInvertPosition = -1; }
        else {              mInvertPosition = 1;  }
    }

    /**
     * Logs the current motor positions, velocities, and power levels.
     *
     * @return A formatted string containing motor telemetry data.
     */
    @Override
    public void                         log()
    {
        if(mConfigurationValid) {
            mLogger.metric(LogManager.Target.DASHBOARD, mName+"-pos","" + mMotor.getCurrentPosition());
            mLogger.metric(LogManager.Target.DASHBOARD, mName+"-spd","" + mMotor.getVelocity());
            mLogger.metric(LogManager.Target.DASHBOARD, mName+"-pwr","" + mMotor.getPower());
        }
    }

    /* ------------------ Configurable functions ------------------- */

    /**
     * Determines if the coupled motor component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /**
     * Reads and applies the motor configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    @Override
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        mMotor = null;

        try {
            if(mMap != null && reader.has(sHwMapKey)) {
                mHwName = reader.getString(sHwMapKey);
                mMotor = mMap.tryGet(DcMotorEx.class,mHwName);
            }

            if(mMotor != null && reader.has(sDirectionKey)) {
                DcMotor.Direction direction = sString2Direction.get(reader.getString(sDirectionKey));
                mMotor.setDirection(direction);
            }
            else if(mMotor != null) {
                mMotor.setDirection(DcMotor.Direction.FORWARD);
            }

            if(mMotor != null && reader.has(sEncoderReverseKey)) {
                boolean shallReverse = reader.getBoolean(sEncoderReverseKey);
                if(shallReverse) { mInvertPosition = -1; }
                else { mInvertPosition = 1; }
            }
            else { mInvertPosition = 1; }

            if(mMotor != null) { mMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

        if (mMotor == null) { mConfigurationValid = false; }

    }

    /**
     * Writes the current motor configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) {

        if(mConfigurationValid) {

            try {
                String direction = sDirection2String.get(mMotor.getDirection());
                writer.put(sHwMapKey, mHwName);
                writer.put(sDirectionKey, direction);
                writer.put(sEncoderReverseKey, mInvertPosition == -1);
            } catch (JSONException e) { mLogger.error(e.getMessage()); }
        }
    }

    /**
     * Generates an HTML representation of the motor configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted motor configuration.
     */
    @Override
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();
        if(mConfigurationValid) {

            if (mMotor != null) {
                result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                        .append("HW : ")
                        .append(mHwName)
                        .append(" - DIR : ")
                        .append(sDirection2String.get(mMotor.getDirection()))
                        .append(" - ENC : ")
                        .append(mInvertPosition == -1)
                        .append("</li>\n");
            }
        }

        return result.toString();

    }

    /**
     * Generates a text-based representation of the motor configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted motor configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();
        if(mConfigurationValid) {

            if (mMotor != null) {
                result.append(header)
                        .append("> HW :")
                        .append(mHwName)
                        .append(" - DIR : ")
                        .append(sDirection2String.get(mMotor.getDirection()))
                        .append(" - ENC : ")
                        .append(mInvertPosition == -1)
                        .append("\n");
            }
        }

        return result.toString();

    }

    /* --------------------- DcMotor functions --------------------- */

    @Override
    public int	                        currentPosition()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = mInvertPosition * mMotor.getCurrentPosition();
        }
        return result;
    }

    @Override
    public DcMotorSimple.Direction      direction()
    {
        DcMotorSimple.Direction result = DcMotorSimple.Direction.FORWARD;
        if(mConfigurationValid) { result = mMotor.getDirection(); }
        return result;
    }

    @Override
    public DcMotor.RunMode	            mode()
    {
        DcMotor.RunMode result =  DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        if (mConfigurationValid) { result = mMotor.getMode(); }
        return result;
    }

    @Override
    public int	                        targetPosition()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = mInvertPosition * mMotor.getTargetPosition();
        }
        return result;
    }

    @Override
    public DcMotor.ZeroPowerBehavior	zeroPowerBehavior()
    {
        DcMotor.ZeroPowerBehavior result = DcMotor.ZeroPowerBehavior.UNKNOWN;
        if(mConfigurationValid) { result = mMotor.getZeroPowerBehavior(); }
        return result;
    }

    @Override
    public double	                    power()
    {
        double result = -1;
        if(mConfigurationValid) { result = mMotor.getPower(); }
        return result;
    }

    @Override
    public boolean	                    isBusy()
    {
        boolean result = false;
        if(mConfigurationValid) { result = mMotor.isBusy(); }
        return result;
    }

    @Override
    public void	                        mode(DcMotor.RunMode mode)
    {
        if(mConfigurationValid) {
            mMotor.setMode(mode);
        }
    }

    @Override
    public void	                        direction(DcMotorSimple.Direction direction)
    {
        if(mConfigurationValid) {
            mMotor.setDirection(direction);
        }
    }

    @Override
    public void	                        targetPosition(int position)
    {
        if(mConfigurationValid) {
            mMotor.setTargetPosition(mInvertPosition * position);
        }
    }

    @Override
    public void	                        zeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        if(mConfigurationValid) {
            mMotor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    @Override
    public void	                        power(double power)
    {
        if(mConfigurationValid) {
            mMotor.setPower(power);
        }
    }
    
    /* -------------------- DcMotorEx functions -------------------- */


    @Override
    public PIDFCoefficients            PIDFCoefficients(DcMotor.RunMode mode){
        PIDFCoefficients result = null;
        if(mConfigurationValid) {
            result = mMotor.getPIDFCoefficients(mode);
        }
        return result;
    }

    @Override
    public void                        PIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients){
        if(mConfigurationValid) {
            mMotor.setPIDFCoefficients(mode, pidfCoefficients);
        }
    }

    @Override
    public void                        targetPositionTolerance(int tolerance)
    {
        if(mConfigurationValid) {
            mMotor.setTargetPositionTolerance(tolerance);
        }
    }

    @Override
    public int                         targetPositionTolerance()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = mMotor.getTargetPositionTolerance();
        }
        return result;
    }

    @Override
    public double                       velocity()
    {
        double result = 0;
        if(mConfigurationValid) {
            result = mMotor.getVelocity();
        }
        return result;

    }



}

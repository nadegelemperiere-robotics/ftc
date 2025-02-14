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

    LogManager          mLogger;

    boolean             mConfigurationValid;
    String              mName;
    String              mHwName;

    HardwareMap         mMap;
    DcMotorEx           mMotor;

    int                 mInvertPosition;

    /* ----------------------- Constructors ------------------------ */

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

    @Override
    public String                       getName() { return mName; }

    @Override
    public boolean                      getEncoderCorrection() { return (mInvertPosition == -1);}

    @Override
    public void                         setEncoderCorrection( boolean ShallCorrect) {
        if (ShallCorrect) { mInvertPosition = -1; }
        else {              mInvertPosition = 1;  }
    }

    @Override
    public String                       logPositions()
    {
        String result = "";
        if(mConfigurationValid) {
            result += "\n  P : " + mMotor.getCurrentPosition() + " V : " + mMotor.getVelocity() + " P : " + mMotor.getPower();
        }
        return result;
    }

    /* ------------------ Configurable functions ------------------- */

    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    @Override
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        mMotor = null;

        try {
            if(reader.has(sHwMapKey)) {
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

    @Override
    public void                         write(JSONObject writer) {

        try {
            String direction = sDirection2String.get(mMotor.getDirection());
            writer.put(sHwMapKey,mHwName);
            writer.put(sDirectionKey,direction);
            writer.put(sEncoderReverseKey, mInvertPosition==-1);
        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

    }

    @Override
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        if (mMotor != null) {
            result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                    .append("HW : ")
                    .append(mHwName)
                    .append(" - DIR : ")
                    .append(sDirection2String.get(mMotor.getDirection()))
                    .append(" - ENC : ")
                    .append(mInvertPosition==-1)
                    .append("</li>\n");
        }

        return result.toString();

    }
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();

        if (mMotor != null) {
            result.append(header)
                    .append("> HW :")
                    .append(mHwName)
                    .append(" - DIR : ")
                    .append(sDirection2String.get(mMotor.getDirection()))
                    .append(" - ENC : ")
                    .append(mInvertPosition==-1)
                    .append("\n");
        }

        return result.toString();

    }

    /* --------------------- DcMotor functions --------------------- */

    @Override
    public int	                        getCurrentPosition()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = mInvertPosition * mMotor.getCurrentPosition();
        }
        return result;
    }

    @Override
    public DcMotorSimple.Direction      getDirection()
    {
        DcMotorSimple.Direction result = DcMotorSimple.Direction.FORWARD;
        if(mConfigurationValid) { result = mMotor.getDirection(); }
        return result;
    }

    @Override
    public DcMotor.RunMode	            getMode()
    {
        DcMotor.RunMode result =  DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        if (mConfigurationValid) { result = mMotor.getMode(); }
        return result;
    }

    @Override
    public int	                        getTargetPosition()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = mInvertPosition * mMotor.getTargetPosition();
        }
        return result;
    }

    @Override
    public DcMotor.ZeroPowerBehavior	getZeroPowerBehavior()
    {
        DcMotor.ZeroPowerBehavior result = DcMotor.ZeroPowerBehavior.UNKNOWN;
        if(mConfigurationValid) { result = mMotor.getZeroPowerBehavior(); }
        return result;
    }

    @Override
    public double	                    getPower()
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
    public void	                        setMode(DcMotor.RunMode mode)
    {
        if(mConfigurationValid) {
            mMotor.setMode(mode);
        }
    }

    @Override
    public void	                        setDirection(DcMotorSimple.Direction direction)
    {
        if(mConfigurationValid) {
            mMotor.setDirection(direction);
        }
    }

    @Override
    public void	                        setTargetPosition(int position)
    {
        if(mConfigurationValid) {
            mMotor.setTargetPosition(mInvertPosition * position);
        }
    }

    @Override
    public void	                        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        if(mConfigurationValid) {
            mMotor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    @Override
    public void	                        setPower(double power)
    {
        if(mConfigurationValid) {
            mMotor.setPower(power);
        }
    }
    
    /* -------------------- DcMotorEx functions -------------------- */


    @Override
    public PIDFCoefficients            getPIDFCoefficients(DcMotor.RunMode mode){
        PIDFCoefficients result = null;
        if(mConfigurationValid) {
            result = mMotor.getPIDFCoefficients(mode);
        }
        return result;
    }

    @Override
    public void                        setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients){
        if(mConfigurationValid) {
            mMotor.setPIDFCoefficients(mode, pidfCoefficients);
        }
    }

    @Override
    public void                        setTargetPositionTolerance(int tolerance)
    {
        if(mConfigurationValid) {
            mMotor.setTargetPositionTolerance(tolerance);
        }
    }

    @Override
    public int                         getTargetPositionTolerance()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = mMotor.getTargetPositionTolerance();
        }
        return result;
    }

    @Override
    public double                      getVelocity()
    {
        double result = 0;
        if(mConfigurationValid) {
            result = mMotor.getVelocity();
        }
        return result;

    }



}

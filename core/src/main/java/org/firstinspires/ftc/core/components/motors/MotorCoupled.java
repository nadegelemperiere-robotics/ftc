/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   CoupledMotor class overloads the FTC motor class to manage
   A couple of motors both turning the same hardware.

   Note that this is a dangerous situation which can result in
   motor destruction if not correctly tuned. The coupled motors
   shall be the same model
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.motors;

/* JSON includes */
import org.json.JSONObject;
import org.json.JSONException;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class MotorCoupled implements MotorComponent {

    public static final String  sFirstKey  = "first";
    public static final String  sSecondKey = "second";

    LogManager                  mLogger;

    String                      mName;
    String                      mFirstHwName;
    String                      mSecondHwName;

    boolean                     mConfigurationValid;
    
    DcMotorSimple.Direction     mDirection;

    HardwareMap                 mMap;
    DcMotorEx                   mFirst;
    DcMotorEx                   mSecond;
    int                         mFirstInvertPosition;
    int                         mSecondInvertPosition;


    /* ----------------------- Constructors ------------------------ */

    public MotorCoupled(String name, HardwareMap hwMap, LogManager logger)
    {
        mLogger                 = logger;
        mName                   = name;
        mFirstHwName            = "";
        mSecondHwName           = "";
        mConfigurationValid     = false;

        mMap                    = hwMap;

        mDirection              = DcMotor.Direction.FORWARD;
        mFirst                  = null;
        mSecond                 = null;
        mFirstInvertPosition    = 1;
        mSecondInvertPosition   = 1;

    }

    /* --------------------- Custom functions ---------------------- */

    @Override
    public String                       getName() { return mName; }

    @Override
    public boolean                      getEncoderCorrection() { return ((mFirstInvertPosition == -1) || (mSecondInvertPosition == -1)); }

    @Override
    public void                         setEncoderCorrection( boolean ShallCorrect) {
        if (mConfigurationValid) {
            if (ShallCorrect) {
                mFirstInvertPosition = -1;
                mSecondInvertPosition = -1;
            } else {
                mFirstInvertPosition = 1;
                mSecondInvertPosition = 1;
            }
        }
    }

    @Override
    public String                       logPositions()
    {
        String result = "";
        if(mConfigurationValid) {
            result += "\n  First : P : " + mFirst.getCurrentPosition() + " V : " + mFirst.getVelocity() + " P : " + mFirst.getPower();
            result += "\n  Second : P : " + mSecond.getCurrentPosition() + " V : " + mSecond.getVelocity() + " P : " + mSecond.getPower();
        }
        return result;
    }

    /* ------------------ Configurable functions ------------------- */

    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    @Override
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        mFirst = null;
        mSecond = null;

        if(!reader.has(sFirstKey))       { mLogger.error("Missing first DC motor for coupled motor"); }
        else if(!reader.has(sSecondKey)) { mLogger.error("Missing second DC motor for coupled motor"); }
        else {
            try {
                JSONObject first = reader.getJSONObject(sFirstKey);
                JSONObject second = reader.getJSONObject(sSecondKey);

                if(first.has(sHwMapKey)) {
                    mFirstHwName = first.getString(sHwMapKey);
                    mFirst = mMap.tryGet(DcMotorEx.class, mFirstHwName);
                }
                if(second.has(sHwMapKey)) {
                    mSecondHwName = second.getString(sHwMapKey);
                    mSecond = mMap.tryGet(DcMotorEx.class, mSecondHwName);
                }

                if(mFirst != null && first.has(sDirectionKey)) {
                    DcMotor.Direction direction = sString2Direction.get(first.getString(sDirectionKey));
                    mFirst.setDirection(direction);
                }
                else if(mFirst != null) {
                    mFirst.setDirection(DcMotor.Direction.FORWARD);
                }
                if(mSecond != null && second.has(sDirectionKey)) {
                    DcMotor.Direction direction = sString2Direction.get(second.getString(sDirectionKey));
                    mSecond.setDirection(direction);
                }
                else if(mSecond != null) {
                    mSecond.setDirection(DcMotor.Direction.FORWARD);
                }

                if(mFirst != null && first.has(sEncoderReverseKey)) {
                    boolean shallReverse = first.getBoolean(sEncoderReverseKey);
                    if(shallReverse) { mFirstInvertPosition = -1; }
                    else { mFirstInvertPosition = 1; }
                }
                else { mFirstInvertPosition = 1; }
                if(mSecond != null && second.has(sEncoderReverseKey)) {
                    boolean shallReverse = second.getBoolean(sEncoderReverseKey);
                    if(shallReverse) { mSecondInvertPosition = -1; }
                    else { mSecondInvertPosition = 1; }
                }
                else { mSecondInvertPosition = 1; }

                if(mFirst != null) { mFirst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }
                if(mSecond != null) { mSecond.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

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
                String direction = sDirection2String.get(mFirst.getDirection());
                first.put(sHwMapKey,mFirstHwName);
                first.put(sDirectionKey,direction);
                first.put(sEncoderReverseKey, mFirstInvertPosition==-1);
            }
            if(mSecond != null) {
                String direction = sDirection2String.get(mSecond.getDirection());
                second.put(sHwMapKey,mSecondHwName);
                second.put(sDirectionKey,direction);
                second.put(sEncoderReverseKey, mSecondInvertPosition==-1);
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
                    .append(" - DIR : ")
                    .append(sDirection2String.get(mFirst.getDirection()))
                    .append(" - ENC : ")
                    .append(mFirstInvertPosition==-1)
                    .append("</li>\n");
        }
        if (mSecond != null) {
            result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                    .append("ID : ")
                    .append(sSecondKey)
                    .append(" - HW : ")
                    .append(mSecondHwName)
                    .append(" - DIR : ")
                    .append(sDirection2String.get(mSecond.getDirection()))
                    .append(" - ENC : ")
                    .append(mSecondInvertPosition==-1)
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
                    .append(" - DIR : ")
                    .append(sDirection2String.get(mFirst.getDirection()))
                    .append(" - ENC : ")
                    .append(mFirstInvertPosition==-1)
                    .append("\n");
        }
        if (mSecond != null) {
            result.append(header)
                    .append("> ")
                    .append(sSecondKey)
                    .append(" HW : ")
                    .append(mSecondHwName)
                    .append(" - DIR : ")
                    .append(sDirection2String.get(mSecond.getDirection()))
                    .append(" - ENC : ")
                    .append(mSecondInvertPosition==-1)
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
            result = (int) (0.5 * mFirstInvertPosition * mFirst.getCurrentPosition() +
                    mSecondInvertPosition * 0.5 * mSecond.getCurrentPosition());
        }
        return result;
    }

    @Override
    public DcMotorSimple.Direction      getDirection()
    {
        return mDirection;
    }


    @Override
    public DcMotor.RunMode	            getMode()
    {
        DcMotor.RunMode result =  DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        if (mConfigurationValid) { result = mFirst.getMode(); }
        return result;
    }

    @Override
    public int	                        getTargetPosition()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = (int) (0.5 * mFirstInvertPosition * mFirst.getTargetPosition() +
                    0.5 * mSecondInvertPosition * mSecond.getTargetPosition());
        }
        return result;
    }

    @Override
    public double	                    getPower()
    {
        double result = -1;
        if(mConfigurationValid) {
            result = (int) (0.5 * mFirst.getPower() + 0.5 * mSecond.getPower());
        }
        return result;
    }

    @Override
    public DcMotor.ZeroPowerBehavior	getZeroPowerBehavior()
    {
        DcMotor.ZeroPowerBehavior result = DcMotor.ZeroPowerBehavior.UNKNOWN;
        if(mConfigurationValid) { result = mFirst.getZeroPowerBehavior(); }
        return result;
    }

    @Override
    public boolean	                    isBusy()
    {
        boolean result = false;
        if(mConfigurationValid) { result = (mFirst.isBusy() || mSecond.isBusy()); }
        return result;
    }

    @Override
    public void	                        setMode(DcMotor.RunMode mode)
    {
        if(mConfigurationValid) {
            mFirst.setMode(mode);
            mSecond.setMode(mode);
        }
    }

    @Override
    public void	                        setDirection(DcMotorSimple.Direction direction)
    {
        if(direction != mDirection && mConfigurationValid) {

            if(     mFirst.getDirection()  == DcMotor.Direction.FORWARD) { mFirst.setDirection(DcMotor.Direction.REVERSE);  }
            else if(mFirst.getDirection()  == DcMotor.Direction.REVERSE) { mFirst.setDirection(DcMotor.Direction.FORWARD);  }

            if(     mSecond.getDirection() == DcMotor.Direction.FORWARD) { mSecond.setDirection(DcMotor.Direction.REVERSE); }
            else if(mSecond.getDirection() == DcMotor.Direction.REVERSE) { mSecond.setDirection(DcMotor.Direction.FORWARD); }

            mDirection = direction;

        }
    }

    @Override
    public void	                        setTargetPosition(int position)
    {
        if(mConfigurationValid) {
            mFirst.setTargetPosition(mFirstInvertPosition * position);
            mSecond.setTargetPosition(mSecondInvertPosition * position);
        }
    }

    @Override
    public void	                        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        if(mConfigurationValid) {
            mFirst.setZeroPowerBehavior(zeroPowerBehavior);
            mSecond.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    @Override
    public void	                        setPower(double power)
    {
        if(mConfigurationValid) {
            mFirst.setPower(power);
            mSecond.setPower(power);
        }
    }

    /* -------------------- DcMotorEx functions -------------------- */

    @Override
    public PIDFCoefficients             getPIDFCoefficients(DcMotor.RunMode mode){
        PIDFCoefficients result = null;
        if(mConfigurationValid) {
            result = mSecond.getPIDFCoefficients(mode);
        }
        return result;
    }

    @Override
    public void                        setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients){
        if(mConfigurationValid) {
            mFirst.setPIDFCoefficients(mode, pidfCoefficients);
            mSecond.setPIDFCoefficients(mode, pidfCoefficients);
        }
    }

    @Override
    public void                        setTargetPositionTolerance(int tolerance)
    {
        if(mConfigurationValid) {
            mFirst.setTargetPositionTolerance(tolerance);
            mSecond.setTargetPositionTolerance(tolerance);
        }
    }

    @Override
    public int                         getTargetPositionTolerance()
    {
        int result = -1;
        if(mConfigurationValid) {
            result = mSecond.getTargetPositionTolerance();
        }
        return result;

    }

    @Override
    public double                       getVelocity()
    {
        double result = 0;
        if(mConfigurationValid) {
            result = 0.5 * mSecond.getVelocity() + 0.5 * mFirst.getVelocity();
        }
        return result;

    }

}

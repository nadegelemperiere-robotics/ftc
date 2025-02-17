/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Default slides subsystem
   ------------------------------------------------------- */
package org.firstinspires.ftc.core.subsystems;

/* JSON object */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class DefaultSlides extends Actuator {

    static final String sMinPosition = "min";
    static final String sMaxPosition = "max";

    int mMinPosition;
    int mMaxPosition;

    /**
     * Constructor
     *
     * @param name The subsystem name
     * @param hardware The robot current hardware
     * @param logger The logger to report events
     */
    public DefaultSlides(String name, Hardware hardware, LogManager logger) {
        super(name, hardware, logger);
    }

    /**
     * Power the slide
     *
     * @param power The power to give to the slide motors
     */
    public void                         power(double power) {

        if(mMotor != null && this.hasFinished()) {
            mMotor.mode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(power < 0 && (mMotor.currentPosition() + mOffset) > mMinPosition) {
                mMotor.power(power);
            }
            else if(power > 0 && (mMotor.currentPosition() + mOffset) < mMaxPosition) {
                mMotor.power(power);
            }
            else {
                mMotor.power(0);
            }
        }

    }

    /**
     * Reads and applies the slides configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    @Override
    public void                         read(JSONObject reader) {
        super.read(reader);

        if(mPositions.containsKey(sMinPosition)) {
            Double position = mPositions.get(sMinPosition);
            if(position != null) { mMinPosition = (int)(double)position; }
        }
        if(mPositions.containsKey(sMaxPosition)) {
            Double position = mPositions.get(sMaxPosition);
            if(position != null) { mMaxPosition = (int)(double)position; }
        }
    }

    /**
     * Writes the current slide configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) {
        try {
            writer.put(sTypeKey, "default-slides");
        }
        catch(JSONException e) {
            mLogger.error(e.getMessage());
        }
        this.writeWithoutType(writer);
    }


}
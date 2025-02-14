/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   A container to mock servo behavior
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.components.servos;

/* System includes */
import static java.lang.Math.max;
import static java.lang.Math.min;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Servo;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;
import org.json.JSONObject;

public class ServoMock implements ServoComponent {

    LogManager                  mLogger;

    boolean                     mConfigurationValid;
    String                      mName;

    ServoControllerComponent    mController;

    Servo.Direction             mDirection;
    double                      mPosition;
    double                      mMin;
    double                      mMax;

    /* -------------- Constructors --------------- */
    public ServoMock(String name, LogManager logger)
    {
        mName   = name;
        mLogger = logger;
        mConfigurationValid  = true;
        mDirection = Servo.Direction.FORWARD;
        mController = new ServoControllerMock(mLogger);
    }

    /* --------------------- Custom functions ---------------------- */

    @Override
    public String                       getName() { return mName; }

    /* ------------------ Configurable functions ------------------- */

    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    @Override
    public void                         read(JSONObject reader) { }

    @Override
    public void                         write(JSONObject writer) { }

    @Override
    public String                       logConfigurationHTML() { return "<p>Mock</p>\n"; }

    @Override
    public String                       logConfigurationText(String header) {   return header + "> Mock"; }


    /* ---------------------- Servo functions ---------------------- */
    @Override
    public ServoControllerComponent     getController() { return mController; }

    @Override
    public Servo.Direction	            getDirection()  { return mDirection;  }

    @Override
    public double	                    getPosition()   { return mPosition;   }

    @Override
    public void	                        scaleRange(double min, double max)
    {
        mMin = min;
        mMax = max;
    }

    @Override
    public void	                        setDirection(Servo.Direction direction) { mDirection = direction; }

    @Override
    public void	                        setPosition(double position)
    {
        mPosition = min(position,mMax);
        mPosition = max(mPosition,mMin);
    }

}

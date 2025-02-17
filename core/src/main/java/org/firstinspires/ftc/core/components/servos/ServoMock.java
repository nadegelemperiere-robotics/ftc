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
import org.firstinspires.ftc.core.orchestration.engine.Mock;
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
        mName                   = name;
        mLogger                 = logger;
        mConfigurationValid     = true;

        mController             = new ServoControllerMock(mLogger);

        mDirection              = Servo.Direction.FORWARD;
        mPosition               = 0;
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
    public void                         read(JSONObject reader) { }

    /**
     * Writes the current servo configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) { }

    /**
     * Generates an HTML representation of the servo configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted servo configuration.
     */
    @Override
    public String                       logConfigurationHTML() { return "<p>Mock</p>\n"; }

    /**
     * Generates a text-based representation of the servo configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted servo configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {   return header + "> Mock\n"; }


    /* ---------------------- Servo functions ---------------------- */
    /**
     * Retrieves the servo controller managing this component.
     *
     * @return The associated ServoControllerComponent.
     */
    @Override
    public ServoControllerComponent     controller() { return mController; }

    /**
     * Retrieves the current direction of the servo.
     *
     * @return The direction of the servo (FORWARD or REVERSE).
     */
    @Override
    public Servo.Direction	            direction()  { return mDirection;  }

    /**
     * Retrieves the position of the servo.
     *
     * @return The servo position in the range [0,1], or -1 if not configured.
     */
    @Override
    public double	                    position()   { return mPosition;   }

    /**
     * Scales the range of motion for the servos.
     *
     * @param min The new minimum position (0.0 to 1.0).
     * @param max The new maximum position (0.0 to 1.0).
     */
    @Override
    public void	                        scaleRange(double min, double max)
    {
        mMin = min;
        mMax = max;
    }

    /**
     * Sets the direction of the servos.
     *
     * @param direction The new direction (FORWARD or REVERSE).
     */
    @Override
    public void	                        direction(Servo.Direction direction) { mDirection = direction; }

    /**
     * Sets the position of the servos.
     *
     * @param position The new position to reach
     */
    @Override
    public void	                        position(double position)
    {
        mPosition = min(position,mMax);
        mPosition = max(mPosition,mMin);
        Mock.instance().mMockedComponents.put(mName + "-position",(double)position);
    }

}

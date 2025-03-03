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

/* JSON includes */
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.Servo;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.InterOpMode;


public class ServoMock implements ServoComponent {

    final LogManager                mLogger;

    final boolean                   mConfigurationValid;
    final String                    mName;

    final ServoControllerComponent  mController;

    Servo.Direction                 mDirection;
    double                          mPosition;
    double                          mMin;
    double                          mMax;

    /* -------------- Constructors --------------- */
    public ServoMock(String name, LogManager logger)
    {
        mName                   = name;
        mLogger                 = logger;
        mConfigurationValid     = true;

        mController             = new ServoControllerMock(mLogger);

        mDirection              = Servo.Direction.FORWARD;
        mPosition               = 0;
        mMin                    = 0.0;
        mMax                    = 1.0;
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
        mLogger.metric(LogManager.Target.DASHBOARD, mName+"-pos","" + mPosition);
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
    public String                       logConfigurationHTML() { return "<p style=\"padding-left:10px; font-size: 11px\">Mock</p>\n"; }

    /**
     * Generates a text-based representation of the servo configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted servo configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {   return header + "> Mock\n"; }

    /* ------------------ HardwareDevice functions ----------------- */

    /**
     * Returns an indication of the manufacturer of this device.
     * @return the manufacturer
     */
    @Override
    public Manufacturer                 getManufacturer() { return Manufacturer.Other; }

    /**
     * Returns a string suitable for display to the user as to the type of device.Note that this is a device-type-specific name; it has nothing to do with the name by which a user might have configured the device in a robot configuration.
     * @return the device name
     */
    @Override
    public String                       getDeviceName() { return "Servo Mock"; }

    /**
     * Get connection information about this device in a human readable format
     * @return connection information
     */
    @Override
    public String                       getConnectionInfo() { return ""; }

    /**
     * Version
     */
    @Override
    public int                          getVersion() { return 0; }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void                         resetDeviceConfigurationForOpMode() {}

    /**
     * Closes this device
     */
    @Override
    public void                         close() {}

    /* ---------------------- Servo functions ---------------------- */
    /**
     * Retrieves the servo controller managing this component.
     *
     * @return The associated ServoControllerComponent.
     */
    @Override
    public ServoControllerComponent     getController() { return mController; }

    /**
     * Unable to provide this method since each motor has a difference port
     * @return -1
     */
    @Override
    public int                          getPortNumber() { return -1; }

    /**
     * Retrieves the current direction of the servo.
     *
     * @return The direction of the servo (FORWARD or REVERSE).
     */
    @Override
    public Servo.Direction	            getDirection()  { return mDirection;  }

    /**
     * Retrieves the position of the servo.
     *
     * @return The servo position in the range [0,1], or -1 if not configured.
     */
    @Override
    public double	                    getPosition()   { return mPosition;   }

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
    public void	                        setDirection(Servo.Direction direction) { mDirection = direction; }

    /**
     * Sets the position of the servos.
     *
     * @param position The new position to reach
     */
    @Override
    public void	                        setPosition(double position)
    {
        mPosition = min(position,mMax);
        mPosition = max(mPosition,mMin);
        InterOpMode.instance().add(mName + "-position",position);
    }

}

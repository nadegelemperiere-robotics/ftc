/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * CameraMock Class
 * -------------------------------------------------------
 * The CameraMock class provides empty camera functionality
 * to enable testing of robot logic without requiring a
 * physical camera component in FTC robots.
 * -------------------------------------------------------
 * Features:
 * - Simulates camera behavior for testing purposes.
 * - Provides methods for generating mock camera frames.
 * - Manages configuration states and logs configuration
 *   details in HTML or text format.
 * -------------------------------------------------------
 */

package org.firstinspires.ftc.core.components.cameras;

/* JSON includes */
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.json.JSONException;
import org.json.JSONObject;

/* OpenCV includes */
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.CvType;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

import java.net.UnknownHostException;

public class CameraMock implements CameraComponent {

    public static final String  sTypeValue  = "mock";

    static final        int     sWidth      = 320;
    static final        int     sHeight     = 240;


    final LogManager            mLogger;

    final boolean               mConfigurationValid;
    final String                mName;

    Mat                         mCurrentFrame;
    int                         mCurrentIndex;

    /* ----------------------- Constructors ------------------------ */
    /**
     * Creates a CameraComponent instance with a specified name and logger.
     *
     * @param name The unique identifier for the camera component.
     * @param logger The logging manager to handle system logs.
     */
    public CameraMock(String name, LogManager logger) {

        mConfigurationValid = true;
        mLogger             = logger;
        mName               = name;

        mCurrentIndex       = 0;
    }

    /**
     * Retrieves the name of the camera component.
     *
     * @return The name of the component.
     */
    public String                       name() { return mName; }

    /**
     * Retrieves the last frame acquired by the camera
     *
     * @return The name of the component.
     */
    public Mat                          current() { return mCurrentFrame; }

    /**
     * Retrieves limelight camera for embedded vision processor access
     *
     * @return A mock limelight
     *
     */
    public Limelight3A                  limelight() {

        Limelight3A result = null;
        try {
            result = new LimelightMock(mLogger);
        }
        catch (UnknownHostException ignored) { }
        return result;
    }
    /**
     * Retrieve the last camera frame
     */
    public void                         update() {

        mCurrentFrame = new Mat(sHeight,sWidth,CvType.CV_8UC3,new Scalar(0));
        Imgproc.putText(mCurrentFrame,""+mCurrentIndex,new Point(20,sHeight - 20), Imgproc.FONT_HERSHEY_COMPLEX,1,new Scalar(255,255,255),2);
        mCurrentIndex ++;
    }

    /* ------------------ Configurable functions ------------------- */

    /**
     * Determines if the camera component has been properly configured.
     *
     * @return True if configuration is valid, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid; }

    /**
     * Reads the camera configuration from a JSON object and initializes the camera.
     *
     * @param reader The JSON object containing configuration settings.
     */
    @Override
    public void                         read(JSONObject reader) { }

    /**
     * Writes the current camera configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) {

        if(mConfigurationValid) {
            try {
                writer.put(sTypeKey, sTypeValue);
            } catch (JSONException e) {
                mLogger.error(e.getMessage());
            }
        }

    }

    /**
     * Generates an HTML representation of the camera configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted camera configuration.
     */
    @Override
    public String                       logConfigurationHTML() { return "<p>Mock</p>\n"; }

    /**
     * Generates a text-based representation of the camera configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted camera configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {  return header + "> Mock\n"; }
}
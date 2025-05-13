/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * CameraDefault provides camera functions from a
 * standard webcam
 * -------------------------------------------------------
 */
package org.firstinspires.ftc.core.components.cameras;

/* Android includes */
import android.graphics.Bitmap;
import android.graphics.Canvas;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC controller includes */
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

/* OpenCV includes */
import org.opencv.android.Utils;
import org.opencv.core.Mat;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class CameraDefault implements CameraComponent {

    public static final String  sTypeKey    = "default";
    static final String sHwMapKey           = "hwmap";


    final LogManager            mLogger;

    boolean                     mConfigurationValid;
    final String                mName;
    String                      mHwName;

    final HardwareMap           mMap;

    VisionPortal                mPortal;
    DisplayProcessor            mProcessor;
    Mat                         mCurrentFrame;

    /* ----------------------- Constructors ------------------------ */
    /**
     * Creates a CameraComponent instance with a specified name and logger.
     *
     * @param name The unique identifier for the camera component.
     * @param logger The logging manager to handle system logs.
     */
    public CameraDefault(String name, HardwareMap hwMap, LogManager logger) {

        mConfigurationValid = true;
        mLogger             = logger;
        mName               = name;

        mHwName             = "";
        mMap                = hwMap;

        mPortal             = null;
        mProcessor          = new DisplayProcessor(mLogger);
        mCurrentFrame       = null;
    }

    /**
     * Retrieves the name of the camera component.
     *
     * @return The name of the component.
     */
    public String                       name()    { return mName; }

    public Mat                          current() { return mCurrentFrame; }

    /**
     * Cache current camera value to enable multiple calls in a loop without
     */
    public void                         update() {
        if (mConfigurationValid) {
            mCurrentFrame = mProcessor.frame().clone();
        }
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
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;
        mPortal = null;
        mHwName = "";

        try {

            if(mMap != null && reader.has(sHwMapKey)) {

                mHwName = reader.getString(sHwMapKey);
                mPortal = new VisionPortal.Builder()
                        .addProcessor(mProcessor)
                        .setCamera(mMap.get(WebcamName.class, mHwName))
                        .build();
            }
        } catch(JSONException e) {
            mLogger.error(e.getMessage());
        }

        if(mPortal == null) { mConfigurationValid = false; }
    }

    /**
     * Writes the current camera configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) {

        if(mConfigurationValid) {
            try {
                writer.put(CameraComponent.sTypeKey, sTypeKey);
                writer.put(sHwMapKey, mHwName);
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
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();
        if (mConfigurationValid) {
            result.append("<p style=\"padding-left:10px; font-size: 11px\"> HW : ")
                    .append(mHwName)
                    .append("</p>");
        }
        return result.toString();
    }

    /**
     * Generates a text-based representation of the camera configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted camera configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();
        if (mConfigurationValid) {
            result.append(header)
                    .append("> HW :")
                    .append(mHwName)
                    .append("\n");
        }
        return result.toString();

    }
}

class DisplayProcessor implements VisionProcessor {

    Mat         mCurrentFrame;
    LogManager  mLogger;

    public DisplayProcessor(LogManager telemetry) {
        mLogger = telemetry;
    }

    @Override
    public void                         init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Mat                          processFrame(Mat frame, long timestamp) {
        mCurrentFrame = frame.clone();
        return frame;
    }

    @Override
    public void                         onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}

    public Mat                          frame() { return mCurrentFrame; }

}
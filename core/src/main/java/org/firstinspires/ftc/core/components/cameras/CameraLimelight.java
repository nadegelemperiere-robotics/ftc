/**
 * -------------------------------------------------------
 * Copyright (c) 2025 Nadege LEMPERIERE
 * All rights reserved
 * -------------------------------------------------------
 * CameraLimelight provides camera functions from a
 * Limelight
 * -------------------------------------------------------
 */

package org.firstinspires.ftc.core.components.cameras;

/* System includes */
import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* OpenCV includes */
import org.opencv.core.Mat;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;


public class CameraLimelight implements CameraComponent {

    public static final String  sTypeKey    = "limelight";
    static final String sHwMapKey           = "hwmap";
    static final String sPortKey            = "port";

    final LogManager            mLogger;

    boolean                     mConfigurationValid;
    final String                mName;
    String                      mHwName;

    final HardwareMap           mMap;

    Limelight3A                 mLimelight;
    int                         mPort;
    String                      mStreamUrl;
    Mat                         mCurrentFrame;

    /* ----------------------- Constructors ------------------------ */
    /**
     * Creates a CameraComponent instance with a specified name and logger.
     *
     * @param name The unique identifier for the camera component.
     * @param logger The logging manager to handle system logs.
     */
    public CameraLimelight(String name, HardwareMap hwMap, LogManager logger) {

        mConfigurationValid = true;
        mLogger             = logger;
        mMap                = hwMap;
        mName               = name;
        mCurrentFrame       = null;
        mStreamUrl          = "";
    }

    /**
     * Retrieves the name of the camera component.
     *
     * @return The name of the component.
     */
    public String                       name() { return mName; }

    /**
     * Cache current camera value to enable multiple calls in a loop without
     */
    public void                         update() {

        try {
            URL url = new URL(mStreamUrl);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestProperty("User-Agent", "OpenCV Java Client");
            conn.setUseCaches(false);

            try (InputStream in = new BufferedInputStream(conn.getInputStream())) {
                ByteArrayOutputStream buffer = new ByteArrayOutputStream();

                boolean insideJPEG = false;
                int b;
                int last = -1;

                while ((b = in.read()) != -1) {
                    if (!insideJPEG) {
                        if (last == 0xFF && b == 0xD8) { // JPEG SOI
                            insideJPEG = true;
                            buffer.write(0xFF);
                            buffer.write(0xD8);
                        }
                    } else {
                        buffer.write(b);

                        // JPEG EOI
                        if (last == 0xFF && b == 0xD9) {
                            break;
                        }
                    }
                    last = b;
                }

                byte[] jpegBytes = buffer.toByteArray();
                MatOfByte mob = new MatOfByte(jpegBytes);
                mCurrentFrame = Imgcodecs.imdecode(mob, Imgcodecs.IMREAD_COLOR);
            }
        } catch (Exception e) {
            mLogger.warning("Failed to fetch or decode frame: " + e.getMessage());
            mCurrentFrame = null;
        }
    }

    public Mat                          current() { return mCurrentFrame; }

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
        mHwName = "";
        mLimelight = null;
        mPort = -1;

        try {

            if(mMap != null && reader.has(sHwMapKey)) {
                mHwName = reader.getString(sHwMapKey);
                mLimelight = mMap.tryGet(Limelight3A.class,mHwName);
            }

            if(reader.has(sPortKey)) {
                mPort = reader.getInt(sPortKey);
            }

            if(mLimelight != null && mPort != -1) {
                String temp = mLimelight.getConnectionInfo();
                mStreamUrl = "http://" + temp.substring(temp.indexOf(':') + 1, temp.length()-1) + ":" + mPort;
            }

        } catch(JSONException e) {
            mLogger.error(e.getMessage());
        }

        if(mLimelight == null) { mConfigurationValid = false; }
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
                writer.put(sPortKey, mPort);
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
                    .append(" - STREAM : ")
                    .append(mStreamUrl)
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
                    .append(" - STREAM : ")
                    .append(mStreamUrl)
                    .append("\n");
        }
        return result.toString();

    }
}
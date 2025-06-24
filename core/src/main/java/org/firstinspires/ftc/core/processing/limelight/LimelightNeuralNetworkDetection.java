/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Limelight neural network detection pipeline management
   ------------------------------------------------------- */
package org.firstinspires.ftc.core.processing.limelight;

/* System includes */
import java.io.IOException;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

/* Components includes */
import org.firstinspires.ftc.core.components.cameras.CameraComponent;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

public class LimelightNeuralNetworkDetection<T extends LimelightObject> extends LimelightPipeline  {

    static  final public    String sTypeKey           = "limelight-nn-detection";
    static  final           String sPipelineKey       = "pipeline";
    static  final           String sIndexKey          = "index";
    static  final public    String sCameraKey         = "camera";
    static  final public    String sPortKey           = "port";

    final LogManager                    mLogger;

    protected boolean                   mConfigurationValid;

    final String                        mName;
    String                              mHwName;

    final Hardware                      mHardware;

    protected Limelight3A               mWebcam;
    final LimelightObjectFactory<T,T>   mFactory;
    int                                 mIndex;
    int                                 mPort;
    String                              mLabels;
    byte[]                              mModel;
    JSONObject                          mPipeline;
    String                              mRestApiUrl;



    /**
     * Constructor
     *
     * @param name The camera name
     * @param hardware The hardware to get camera from
     * @param logger The logger to use for traces
     */
    public LimelightNeuralNetworkDetection(String name, LimelightObjectFactory<T,T> factory, byte[] model, String labels, Hardware hardware, LogManager logger) {

        mLogger             = logger;

        mConfigurationValid = false;
        mName               = name;

        mHardware           = hardware;

        mIndex              = -1;
        mPort               = -1;
        mModel              = model;
        mLabels             = labels;
        mRestApiUrl         = "";
        mPipeline           = new JSONObject();

        mWebcam             = null;
        mFactory            = factory;
    }

    public String                           name() { return mName; }

    /**
     * Start camera streaming
     */
    public void                             start() {

        if(mConfigurationValid) {

            mLogger.info("starting neural network detection pipeline");
            mWebcam.pipelineSwitch(mIndex);
            mWebcam.start();
        }
    }

    /**
     * Retrieve new results
     *
     * @return List of detected samples
     */
    public List<T>                          process() {

        List<T> result = new ArrayList<>();

        if(mConfigurationValid) {

            LLResult results = mWebcam.getLatestResult();

            if(results != null) {

                List<LLResultTypes.DetectorResult> detections = results.getDetectorResults();
                for (int i_sample = 0; i_sample < detections.size(); i_sample++) {
                    T object = mFactory.build(mName,detections.get(i_sample));
                    result.add(object);
                }
            }

        }

        return result;
    }


    /* ------------------ Configurable functions ------------------- */

    /**
     * Determines if the limelight component is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    @Override
    public boolean                      isConfigured() { return mConfigurationValid;}

    /**
     * Reads and applies the limelight configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    @Override
    public void                         read(JSONObject reader) {

        mConfigurationValid = true;

        mWebcam             = null;
        mIndex              = -1;
        mPipeline           = new JSONObject();

        try {

            if(mHardware != null && reader.has(sCameraKey)) {
                mHwName = reader.getString(sCameraKey);
                CameraComponent component = null;
                Map<String, CameraComponent> cameras = mHardware.cameras();
                if(cameras.containsKey(mHwName)) { component = cameras.get(mHwName); }
                if(component != null) { mWebcam = component.limelight();}
            }

            if (reader.has(sIndexKey)) {
                mIndex = reader.getInt(sIndexKey);
            }

            if(reader.has(sPipelineKey)) {
                mPipeline = reader.getJSONObject(sPipelineKey);
            }

            if (reader.has(sPortKey)) {
                mPort = reader.getInt(sPortKey);
            }
        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

        if (mWebcam == null) {
            mLogger.error("No camera found or wrong parameter");
            mConfigurationValid = false;
        }
        if ( mIndex < 0 || mIndex > 7) {
            mLogger.error("Invalid pipeline identifier : " + mIndex);
            mConfigurationValid = false;
        }
        if( mPort == -1) {
            mLogger.error("Invalid port : " + mPort);
            mConfigurationValid = false;
        }

        if(mConfigurationValid) {
            if(mModel == null) { mConfigurationValid = false; }
            else {
                String temp = mWebcam.getConnectionInfo();
                mRestApiUrl = "http://" + temp.substring(temp.indexOf(':') + 1, temp.length()-1) + ":" + mPort;

                boolean check = this.uploadPipeline(mPipeline, mRestApiUrl, mIndex, mLogger);
                if (!check) {
                    mLogger.error("Could not update pipeline : " + mName);
                    mConfigurationValid = false;
                }
                check = this.uploadDetectorLabels(mLabels,mRestApiUrl,mIndex,mLogger);
                if (!check) {
                    mLogger.error("Could not update labels for pipeline : " + mName);
                    mConfigurationValid = false;
                }
//                check = this.uploadModel();
//                if (!check) {
//                    mLogger.error("Could not update model for pipeline : " + mName);
//                    mConfigurationValid = false;
//                }
            }
        }

    }

    /**
     * Writes the current limelight configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    @Override
    public void                         write(JSONObject writer) {

        if(mConfigurationValid) {

            try {
                writer.put(sCameraKey,mHwName);
                writer.put(sIndexKey,mIndex);
                writer.put(sPipelineKey,mPipeline);

            } catch (JSONException e) { mLogger.error(e.getMessage()); }
        }

    }

    /**
     * Generates an HTML representation of the limelight configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted limelight configuration.
     */
    @Override
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {

            result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                    .append(" - HW : ")
                    .append(mHwName)
                    .append(" PPL : ")
                    .append(mIndex)
                    .append(" - STREAM : ")
                    .append(mRestApiUrl)
                    .append("</li>");

        }

        return result.toString();

    }

    /**
     * Generates a text-based representation of the limelight configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted limelight configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();

        if (mConfigurationValid) {

            result.append(header)
                    .append("> HW : ")
                    .append(mHwName)
                    .append(" PPL : ")
                    .append(mIndex)
                    .append(" - STREAM : ")
                    .append(mRestApiUrl);
        }

        return result.toString();
    }



}

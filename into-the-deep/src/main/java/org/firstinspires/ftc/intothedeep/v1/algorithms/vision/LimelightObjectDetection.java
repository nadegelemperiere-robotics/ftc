/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Limelight sample detection pipeline management
   ------------------------------------------------------- */
package org.firstinspires.ftc.intothedeep.v1.algorithms.vision;

/* System includes */
import java.util.ArrayList;
import java.util.List;
import java.util.Comparator;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

public class LimelightObjectDetection implements Configurable {


    static  final public    String sTypeKey           = "detection";
    static  final public    String sPipelineKey       = "pipeline";
    static  final public    String sHwMapKey          = "hwmap";

    final LogManager                mLogger;

    protected boolean               mConfigurationValid;

    final String                    mName;
    String                          mHwName;

    final HardwareMap               mMap;

    protected Limelight3A           mWebcam;
    int                             mPipeline;
    int                             mSampleId;


    /**
     * Constructor
     *
     * @param name The camera name
     * @param map The hardware map to get sensors from
     * @param logger The logger to use for traces
     */
    public  LimelightObjectDetection(String name, HardwareMap map, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;
        mName               = name;

        mMap                = map;

        mWebcam             = null;
        mPipeline           = -1;

        // We start at one to avoid mixing receiving null data with receiving update for
        // sample 0
        mSampleId           = 1;

    }

    /**
     * Start camera streaming
     */
    public void                         start() {

        if (mConfigurationValid) {
            mLogger.info("starting object detection pipeline");
            mWebcam.pipelineSwitch(mPipeline);
            mWebcam.start();
        }

    }

    /**
     * Retrieve new results
     *
     * @return List of detected samples
     */
    public List<Sample>                     process() {

        List<Sample> result = new ArrayList<>();

        if (mConfigurationValid) {

            LLResult results = mWebcam.getLatestResult();

            if(results != null) {

                List<LLResultTypes.DetectorResult> detections = results.getDetectorResults();
                for (int i_sample = 0; i_sample < detections.size(); i_sample++) {
                    Sample sample = new Sample(mSampleId, detections.get(i_sample), mLogger);
                    result.add(sample);
                    mSampleId ++;
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
        mPipeline           = -1;

        try {

            if (mMap != null && reader.has(sHwMapKey)) {
                mHwName = reader.getString(sHwMapKey);
                mWebcam = mMap.get(Limelight3A.class, mHwName);
            }

            if (mWebcam != null) {
                if (reader.has(sPipelineKey)) {
                    mPipeline = reader.getInt(sPipelineKey);
                }
            }
        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

        if (mWebcam == null) { mConfigurationValid = false; }

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

                writer.put(sHwMapKey,mHwName);
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
                    .append(mPipeline)
                    .append("</li>\n");

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
                    .append(mPipeline)
                    .append("\n");
        }

        return result.toString();
    }

}

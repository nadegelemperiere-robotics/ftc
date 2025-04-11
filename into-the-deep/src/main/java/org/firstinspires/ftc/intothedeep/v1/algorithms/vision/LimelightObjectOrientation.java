/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Limelight sample orientation pipeline management
   ------------------------------------------------------- */
package org.firstinspires.ftc.intothedeep.v1.algorithms.vision;

/* System includes */
import java.util.ArrayList;
import java.util.List;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class LimelightObjectOrientation implements Configurable {
    static  final public    String sTypeKey           = "orientation";
    static  final           String sPipelineKey       = "pipeline";
    static  final private   String sUpdateColorKey    = "update-color";
    static  final public    String sHwMapKey          = "hwmap";
    private static final Logger log = LoggerFactory.getLogger(LimelightObjectOrientation.class);

    final LogManager                mLogger;

    protected boolean               mConfigurationValid;

    final String                    mName;
    String                          mHwName;

    final HardwareMap               mMap;

    protected Limelight3A           mWebcam;
    int                             mPipeline;
    int                             mLastProcessed;

    boolean                         mShallUpdateColor;

    List<Sample>                    mWaitingList;

    /**
     * Constructor
     *
     * @param name The camera name
     * @param map The hardware map to get sensors from
     * @param logger The logger to use for traces
     */
    public  LimelightObjectOrientation(String name, HardwareMap map, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;
        mName               = name;

        mMap                = map;

        mWebcam             = null;
        mPipeline           = -1;
        mLastProcessed      = 0;

        mShallUpdateColor   = true;

    }

    /**
     * Start camera streaming
     */
    public void                         start(List<Sample> samples) {

        if (mConfigurationValid) {

            mLogger.info("starting object orientation pipeline");

            // Format samples into pipeline inputs
            double[] data = new double[samples.size() * 7 + 1];

            mLogger.info(""+samples.size());
            data[0] = mLastProcessed;
            int i_data = 1;
            for (int i_sample = 0; i_sample < samples.size(); i_sample++) {

                Sample sample = samples.get(i_sample);
                double size = Math.sqrt(sample.area() * 2.33);

                Sample.Color color = sample.color();
                int col = -1;
                if (color == Sample.Color.RED)         { col = 0; }
                else if (color == Sample.Color.BLUE)   { col = 1;}
                else if (color == Sample.Color.YELLOW) { col = 2; }

                mLogger.info("" + sample.index());

                data[i_data] = sample.index(); i_data++;
                data[i_data] = sample.x(); i_data++;
                data[i_data] = sample.y(); i_data++;
                data[i_data] = size; i_data++;
                data[i_data] = size; i_data++;
                data[i_data] = col; i_data++;
                data[i_data] = sample.area(); i_data ++;
            }

            mLogger.info(""+data.length);

            mWebcam.pipelineSwitch(mPipeline);
            mWebcam.start();
            mWebcam.updatePythonInputs(data);
            mWaitingList = samples;
        }
    }

    /**
     * Retrieve new results
     *
     * @return List of updated samples
     */
    public List<Sample>                     process() {

        List<Sample> result = new ArrayList<>();

        if (mConfigurationValid) {

            LLResult results = mWebcam.getLatestResult();
            if(results != null) {
                double[] orientations = results.getPythonOutput();

                if (orientations[0] == mLastProcessed) {
                    for (int i_sample = 0; i_sample < (int) ((orientations.length - 1) / 10); i_sample++) {

                        int index = (int) (orientations[i_sample * 10 + 1]);
                        if(index != 0) {
                            mLogger.info("" + index);

                            double orientation = orientations[i_sample * 10 + 5];
                            int col = (int) orientations[i_sample * 10 + 4];

                            Sample.Color color = Sample.Color.UNKNOWN;
                            if (col == 0) {
                                color = Sample.Color.RED;
                            } else if (col == 1) {
                                color = Sample.Color.BLUE;
                            } else if (col == 2) {
                                color = Sample.Color.YELLOW;
                            }

                            for (int j_sample = 0; j_sample < mWaitingList.size(); j_sample++) {
                                mLogger.info("" + mWaitingList.get(j_sample).index());
                                if (index == mWaitingList.get(j_sample).index()) {
                                    if (mShallUpdateColor) {
                                        mWaitingList.get(j_sample).color(color);
                                    }
                                    mWaitingList.get(j_sample).orientation(orientation);
                                }
                            }
                        }

                    }

                    mLastProcessed ++;
                    result = mWaitingList;

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
        mShallUpdateColor   = true;

        try {

            if (mMap != null && reader.has(sHwMapKey)) {
                mHwName = reader.getString(sHwMapKey);
                mWebcam = mMap.get(Limelight3A.class, mHwName);
            }

            if (mWebcam != null) {
                if (reader.has(sPipelineKey)) {
                    mPipeline = reader.getInt(sPipelineKey);
                }
                if(reader.has(sUpdateColorKey)) {
                    mShallUpdateColor = reader.getBoolean(sUpdateColorKey);
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

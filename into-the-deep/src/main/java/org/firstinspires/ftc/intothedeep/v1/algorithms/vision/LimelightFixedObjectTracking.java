/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Limelight fixed objet tracking pipeline management
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


public class LimelightFixedObjectTracking implements Configurable {

    static  final public    String sTypeKey           = "tracking";
    static  final public    String sPipelineKey       = "pipeline";
    static  final public    String sHwMapKey          = "hwmap";

    final LogManager                mLogger;

    protected boolean               mConfigurationValid;

    final String                    mName;
    String                          mHwName;

    final HardwareMap               mMap;

    protected Limelight3A           mWebcam;
    int                             mPipeline;
    int                             mLastProcessed;

    List<Sample>                    mWaitingList;
    /**
     * Constructor
     *
     * @param name The camera name
     * @param map The hardware map to get sensors from
     * @param logger The logger to use for traces
     */
    public  LimelightFixedObjectTracking(String name, HardwareMap map, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;
        mName               = name;

        mMap                = map;

        mWebcam             = null;
        mPipeline           = -1;
        mLastProcessed      = -1;

    }

    /**
     * Start camera pipeline with new input
     */
    public void                         start(int index, List<Sample> samples) {

        if (mConfigurationValid) {

            mLogger.info("starting object detection pipeline");

            // Format samples into pipeline inputs
            double[] data = new double[samples.size() * 10 + 1];
            int i_data = 0;
            data[0] = index;
            for (int i_sample = 0; i_sample < samples.size(); i_sample++) {

                Sample sample = samples.get(i_sample);

                Sample.Color color = sample.color();
                int col = -1;
                if (color == Sample.Color.RED)         { col = 0; }
                else if (color == Sample.Color.BLUE)   { col = 1;}
                else if (color == Sample.Color.YELLOW) { col = 2; }

                data[i_data] = sample.index(); i_data++;
                data[i_data] = sample.x(); i_data++;
                data[i_data] = sample.y(); i_data++;
                data[i_data] = sample.area(); i_data++;
                data[i_data] = col; i_data++;
                data[i_data] = sample.orientation(); i_data++;
                data[i_data] = sample.xMin(); i_data ++;
                data[i_data] = sample.xMax(); i_data ++;
                data[i_data] = sample.yMin(); i_data ++;
                data[i_data] = sample.yMax(); i_data ++;
            }


            if (!mWebcam.isRunning()) { mWebcam.start(); }
            mWebcam.pipelineSwitch(mPipeline);
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
                double[] tracks = results.getPythonOutput();

                if (tracks[0] != mLastProcessed) {
                    for (int i_sample = 0; i_sample < (int) ((tracks.length - 1) / 11); i_sample++) {

                        int index = (int) (tracks[i_sample * 11 + 1]);
                        double x = tracks[i_sample * 11 + 3];
                        double y = tracks[i_sample * 11 + 4];
                        int col = (int) tracks[i_sample * 11 + 5];
                        double orientation = tracks[i_sample * 11 + 6];
                        double area = tracks[i_sample * 11 + 7];
                        double xmin = tracks[i_sample * 11 + 8];
                        double xmax = tracks[i_sample * 11 + 9];
                        double ymin = tracks[i_sample * 11 + 10];
                        double ymax = tracks[i_sample * 11 + 11];

                        Sample.Color color = Sample.Color.UNKNOWN;
                        if (col == 0) {
                            color = Sample.Color.RED;
                        } else if (col == 1) {
                            color = Sample.Color.BLUE;
                        } else if (col == 2) {
                            color = Sample.Color.YELLOW;
                        }

                        for (int j_sample = 0; j_sample < mWaitingList.size(); j_sample++) {
                            if (index == mWaitingList.get(j_sample).index()) {
                                mWaitingList.get(j_sample).x(x);
                                mWaitingList.get(j_sample).y(y);
                                mWaitingList.get(j_sample).orientation(orientation);
                            }
                        }

                    }

                    mLastProcessed = (int) tracks[0];
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

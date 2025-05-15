/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Limelight sample orientation pipeline management
   ------------------------------------------------------- */
package org.firstinspires.ftc.core.processing.limelight;

/* System includes */
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

/* Components includes */
import org.firstinspires.ftc.core.components.cameras.CameraComponent;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

public class LimelightPythonSnapscript<T extends LimelightObject, U extends LimelightObject> implements Configurable {

    static  final public    String sTypeKey           = "limelight-snapscript";
    static  final           String sPipelineKey       = "pipeline";
    static  final public    String sCameraKey         = "camera";

    final LogManager                mLogger;

    protected boolean               mConfigurationValid;

    final String                    mName;
    String                          mHwName;

    final Hardware                  mHardware;

    Limelight3A                     mWebcam;
    int                             mPipeline;
    LimelightObjectFactory<T,U>     mFactory;
    int                             mLastProcessed;
    String                          mCode;

    /**
     * Constructor
     *
     * @param name The camera name
     * @param code The pipeline python code
     * @param hardware The hardware to get camera from
     * @param logger The logger to use for traces
     */
    public LimelightPythonSnapscript(String name, LimelightObjectFactory<T,U> factory, String code, Hardware hardware, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;
        mName               = name;

        mHardware           = hardware;

        mWebcam             = null;
        mFactory            = factory;
        mCode               = code;
        mPipeline           = -1;
        mLastProcessed      = 1;

    }

    /**
     * Start camera streaming
     */
    public void                         start(List<T> inputs) {

        if (mConfigurationValid) {

            mLogger.info("starting python snapscript pipeline with " + inputs.size() + " inputs");

            // Format samples into pipeline inputs
            double[] data = new double[1];
            data[0] = mLastProcessed;
            for (T object : inputs) {
                double[] temp = mFactory.format(mName, object);
                double[] temp2 = new double[data.length + temp.length];
                System.arraycopy(data,0,temp2,0,data.length);
                System.arraycopy(temp,0,temp2,data.length,temp.length);
                data = temp2;
            }

            mWebcam.pipelineSwitch(mPipeline);
            mWebcam.start();
            mWebcam.updatePythonInputs(data);
        }
    }

    /**
     * Retrieve new results
     *
     * @return List of updated samples
     */
    public List<U>                      process() {

        List<U> result = new ArrayList<>();

        if (mConfigurationValid) {

            LLResult results = mWebcam.getLatestResult();
            if(results != null) {
                double[] data = results.getPythonOutput();
                int increment = mFactory.increment(mName);

                if (data[0] == mLastProcessed) {
                    for (int i_sample = 0; i_sample < ((data.length - 1) / increment); i_sample++) {

                        U object = mFactory.build(mName, data, i_sample * increment + 1);
                        mLogger.debug(""+object);
                        if(object != null)  { result.add(object); }
                    }
                    mLastProcessed ++;
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

            if(mHardware != null && reader.has(sCameraKey)) {
                mHwName = reader.getString(sCameraKey);
                CameraComponent component = null;
                Map<String, CameraComponent> cameras = mHardware.cameras();
                if(cameras.containsKey(mHwName)) { component = cameras.get(mHwName); }
                if(component != null) { mWebcam = component.limelight();}
            }

            if (reader.has(sPipelineKey)) {
                mPipeline = reader.getInt(sPipelineKey);
            }
        }
        catch(JSONException e) { mLogger.error(e.getMessage()); }

        if (mWebcam == null) {
            mLogger.error("No camera found or wrong parameter");
            mConfigurationValid = false;
        }
        if ( mPipeline < 0 || mPipeline > 7) {
            mLogger.error("Invalid pipeline identifier : " + mPipeline);
            mConfigurationValid = false;
        }

        if(mConfigurationValid) {
            boolean check = mWebcam.uploadPython(mCode, mPipeline);
            if(!check) {
                mLogger.error("Could not update code for pipeline : " + mName);
                mConfigurationValid = false;
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

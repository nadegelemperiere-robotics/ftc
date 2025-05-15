/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Limelight sample orientation pipeline management
   ------------------------------------------------------- */
package org.firstinspires.ftc.core.processing.limelight;

/* System includes */
import java.io.IOException;
import java.io.OutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.BufferedReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.net.HttpURLConnection;
import java.net.URL;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONArray;

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
    static  final public    String sPortKey           = "port";

    final LogManager                    mLogger;

    protected boolean                   mConfigurationValid;

    final String                        mName;
    String                              mHwName;

    final Hardware                      mHardware;

    Limelight3A                         mWebcam;
    int                                 mPipeline;
    final LimelightObjectFactory<T,U>   mFactory;
    int                                 mLastProcessed;
    final String                        mCode;
    String                              mRestApiUrl;
    int                                 mPort;

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
        mRestApiUrl         = "";
        mPort               = -1;

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
            //mWebcam.updatePythonInputs(data);
            this.sendData(data);
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

            double[] data = this.getData();
            if(data != null) {
                int increment = mFactory.increment(mName);

                if (data[0] == mLastProcessed) {
                    for (int i_sample = 0; i_sample < ((data.length - 1) / increment); i_sample++) {

                        U object = mFactory.build(mName, data, i_sample * increment + 1);
                        if(object != null)  { result.add(object); }
                    }
                    mLastProcessed ++;
                }

            }
        }

        return result;
    }

    /**
     * Bypass updatePythonInputs, which is limited to 32 data by attacking
     * directly the limelight rest API which does not suffer such a limitation
     * @param data the input data for python pipeline
     */
    void                                sendData(double[] data) {
        if(mConfigurationValid) {
            try {
                URL url = new URL(mRestApiUrl + "/update-pythoninputs");
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestMethod("POST");
                conn.setDoOutput(true);
                conn.setRequestProperty("Content-Type", "application/json");

                JSONArray jsonArray = new JSONArray(data);

                OutputStream out = conn.getOutputStream();
                out.write(jsonArray.toString().getBytes(StandardCharsets.UTF_8));

                if (conn.getResponseCode() != 200) {
                    throw new IOException("Failed to send Python input: " + conn.getResponseCode());
                }
            }
            catch (IOException | JSONException e) {
                mLogger.warning("Could not send python pipeline data : " + e);
            }
        }


    }

    /**
     * Bypass getPythonOutput, which is limited to 32 data by attacking
     * directly the limelight rest API which does not suffer such a limitation
     * @return The output data of the pipeline
     */
    double[]                            getData() {

        double[] result = null;

        if(mConfigurationValid) {

            try {
                URL url = new URL(mRestApiUrl + "/results");
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestMethod("GET");

                InputStream in = conn.getInputStream();
                InputStreamReader reader = new InputStreamReader(in, StandardCharsets.UTF_8);
                BufferedReader br = new BufferedReader(reader);

                StringBuilder response = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) {
                    response.append(line);
                }

                JSONObject json = new JSONObject(response.toString());
                JSONArray pythonOut = json.getJSONArray("PythonOut");

                result = new double[pythonOut.length()];
                for (int i = 0; i < pythonOut.length(); i++) {
                    result[i] = pythonOut.getDouble(i);
                }
            }
            catch (IOException | JSONException e) {
                mLogger.warning("Could not send python pipeline data : " + e);
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
        mRestApiUrl         = "";

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

            if(reader.has(sPortKey)) {
                mPort = reader.getInt(sPortKey);
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
        if( mPort == -1) {
            mLogger.error("Invalid port : " + mPort);
            mConfigurationValid = false;
        }

        if(mConfigurationValid) {
            boolean check = mWebcam.uploadPython(mCode, mPipeline);
            if(!check) {
                mLogger.error("Could not update code for pipeline : " + mName);
                mConfigurationValid = false;
            }
        }

        if(mConfigurationValid) {
            String temp = mWebcam.getConnectionInfo();
            mRestApiUrl = "http://" + temp.substring(temp.indexOf(':') + 1, temp.length()-1) + ":" + mPort;

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
                writer.put(sPortKey,mPort);

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
                    .append(" - STREAM : ")
                    .append(mRestApiUrl)
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
                    .append(" - STREAM : ")
                    .append(mRestApiUrl)
                    .append("\n");
        }

        return result.toString();
    }

}

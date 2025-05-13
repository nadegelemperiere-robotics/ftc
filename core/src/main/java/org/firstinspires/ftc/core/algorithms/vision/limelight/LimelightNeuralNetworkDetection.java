///* -------------------------------------------------------
//   Copyright (c) [2025] Nadege LEMPERIERE
//   All rights reserved
//   -------------------------------------------------------
//   Limelight sample detection pipeline management
//   ------------------------------------------------------- */
//package org.firstinspires.ftc.core.algorithms.vision;
//
///* System includes */
//import java.util.ArrayList;
//import java.util.List;
//
///* JSON includes */
//import org.json.JSONException;
//import org.json.JSONObject;
//
///* Qualcomm includes */
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//
///* Tools includes */
//import org.firstinspires.ftc.core.tools.LogManager;
//
///* Configuration includes */
//import org.firstinspires.ftc.core.configuration.Configurable;
//
///* Components includes */
//import org.firstinspires.ftc.core.components.cameras.CameraComponent;
//import org.firstinspires.ftc.core.components.cameras.CameraLimelight;
//
///* Robot includes */
//import org.firstinspires.ftc.core.robot.Hardware;
//
//public class LimelightNeuralNetworkDetection<T extends LimelightNeuralNetworkOutput>  implements Configurable, LimelightPipeline {
//
//    static  final public    String sTypeKey           = "limelight-nn-detection";
//    static  final           String sPipelineKey       = "pipeline";
//
//    static  final public    String sHwMapKey          = "hwmap";
//
//    final LogManager                mLogger;
//
//    protected boolean               mConfigurationValid;
//
//    final String                    mName;
//    String                          mHwName;
//
//    final Hardware                  mHardware;
//
//    protected Limelight3A           mWebcam;
//    int                             mPipeline;
//    int                             mSampleId;
//
//    /**
//     * Constructor
//     *
//     * @param name The camera name
//     * @param hardware The hardware to get camera from
//     * @param logger The logger to use for traces
//     */
//    public LimelightNeuralNetworkDetection(String name, Hardware hardware, LogManager logger) {
//
//        mLogger             = logger;
//
//        mConfigurationValid = false;
//        mName               = name;
//
//        mHardware           = hardware;
//
//        mWebcam             = null;
//        mPipeline           = -1;
//        mSampleId           = 0;
//
//    }
//
//    /**
//     * Start camera streaming
//     */
//    public void                             start() {
//
//        if(mConfigurationValid) {
//
//            mLogger.info("starting neural network detection pipeline");
//            mWebcam.pipelineSwitch(mPipeline);
//            mWebcam.start();
//        }
//    }
//
//    /**
//     * Retrieve new results
//     *
//     * @return List of detected samples
//     */
//    public List<T>                          process() {
//
//        List<T> result = new ArrayList<>();
//
//        if(mConfigurationValid) {
//
//            LLResult results = mWebcam.getLatestResult();
//
//            if(results != null) {
//
//                List<LLResultTypes.DetectorResult> detections = results.getDetectorResults();
//                for (int i_sample = 0; i_sample < detections.size(); i_sample++) {
//                    T object = (T) T.build(mSampleId,detections.get(i_sample), mLogger);
//                    result.add(object);
//                    mSampleId ++;
//                }
//            }
//
//        }
//
//        return result;
//    }
//
//
//    /* ------------------ Configurable functions ------------------- */
//
//    /**
//     * Determines if the limelight component is configured correctly.
//     *
//     * @return True if the component is configured, false otherwise.
//     */
//    @Override
//    public boolean                      isConfigured() { return mConfigurationValid;}
//
//    /**
//     * Reads and applies the limelight configuration from a JSON object.
//     *
//     * @param reader The JSON object containing configuration settings.
//     */
//    @Override
//    public void                         read(JSONObject reader) {
//
//        mConfigurationValid = true;
//
//        mWebcam             = null;
//        mPipeline           = -1;
//
//        try {
//
//            if (mHardware != null && reader.has(sHwMapKey)) {
//                mHwName = reader.getString(sHwMapKey);
//                CameraComponent temp =  mHardware.cameras().get(mHwName);
//                if (temp instanceof CameraLimelight) {
//                    mWebcam = null;
//                    if(mWebcam == null) { mLogger.error("Did not get any raw webcam data"); }
//                }
//                else { mLogger.error("Camera is not a limelight"); }
//            }
//
//            if (mWebcam != null) {
//                if (reader.has(sPipelineKey)) {
//                    mPipeline = reader.getInt(sPipelineKey);
//                }
//            }
//        }
//        catch(JSONException e) { mLogger.error(e.getMessage()); }
//
//        if (mWebcam == null) {
//            mLogger.error("No camera found or wrong parameter");
//            mConfigurationValid = false; }
//
//    }
//
//    /**
//     * Writes the current limelight configuration to a JSON object.
//     *
//     * @param writer The JSON object to store the configuration settings.
//     */
//    @Override
//    public void                         write(JSONObject writer) {
//
//        if(mConfigurationValid) {
//
//            try {
//                writer.put(sHwMapKey,mHwName);
//                writer.put(sPipelineKey,mPipeline);
//
//            } catch (JSONException e) { mLogger.error(e.getMessage()); }
//        }
//
//    }
//
//    /**
//     * Generates an HTML representation of the limelight configuration for logging purposes.
//     *
//     * @return A string containing the HTML-formatted limelight configuration.
//     */
//    @Override
//    public String                       logConfigurationHTML() {
//
//        StringBuilder result = new StringBuilder();
//
//        if(mConfigurationValid) {
//
//            result.append("<li style=\"padding-left:10px; font-size: 11px\">")
//                    .append(" - HW : ")
//                    .append(mHwName)
//                    .append(" PPL : ")
//                    .append(mPipeline)
//                    .append("</li>");
//
//        }
//
//        return result.toString();
//
//    }
//
//    /**
//     * Generates a text-based representation of the limelight configuration for logging.
//     *
//     * @param header A string to prepend to the configuration log.
//     * @return A string containing the formatted limelight configuration details.
//     */
//    @Override
//    public String                       logConfigurationText(String header) {
//
//        StringBuilder result = new StringBuilder();
//
//        if (mConfigurationValid) {
//
//            result.append(header)
//                    .append("> HW : ")
//                    .append(mHwName)
//                    .append(" PPL : ")
//                    .append(mPipeline);
//        }
//
//        return result.toString();
//    }
//}

/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Into-The-Deep Sample detection processing mock function
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.processing;

/* System includes */
import java.util.List;
import java.util.ArrayList;

/* JSON includes */
import org.json.JSONObject;

/* Opencv includes */
import org.opencv.core.Mat;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

public class SamplesDetectionMock extends SamplesDetection {

    static final String         sTypeValue = "sample-detection-mock";

    final LogManager                                mLogger;

    boolean                                         mConfigurationValid;
    final String                                    mName;

    List<Sample>                                    mOngoing;
    List<Sample>                                    mConsolidated;
    Sample                                          mSelected;

    /**
     * Constructor
     *
     * @param name The camera name
     * @param hardware The hardware data to get sensors from
     * @param logger The logger to use for traces
     */
    public SamplesDetectionMock(String name, LogManager logger) {

        mLogger             = logger;

        mName               = name;

        mConfigurationValid = true;

        mConsolidated       = new ArrayList<>();
        mOngoing            = new ArrayList<>();
        mSelected           = null;

    }

    public void                             color(Sample.Color color) {  }

    public String                           name() { return mName; }

    public List<Sample>                     samples() { return mConsolidated; }

    public boolean                          isConfigured() { return mConfigurationValid; }

    /**
     * Start sample detection
     */
    public void                             start() {
    }

    /**
     * Detect new samples
     */
    public void                             update() {
        mImageIndex++;
    }

    /**
     * Add overlay to the raw image
     *
     * @param raw : raw frame to draw overlays on
     *
     * @return The camera image with samples overlays
     * **/
    public Mat                           draw(Mat raw) {

        Mat result = null;

        if(raw != null) {

            result = raw.clone();
        }

        return result;
    }


    public void                         read(JSONObject reader) {
        mConfigurationValid = true;
    }

    public void                         write(JSONObject writer) {
    }


    /** Log sample detection results **/
    public void                         log(String header)
    {

        StringBuilder result = new StringBuilder();

        result.append("<p> Mock sample detection </p> ");
        mLogger.raw(LogManager.Target.DASHBOARD, result.toString());

        result = new StringBuilder();

        result.append(header);
        result.append("> MOCKED\n");

        mLogger.info(LogManager.Target.FILE,result.toString());
    }

    /** Log consolidated samples to the dashboard **/
    public String                       logConfigurationText(String header)
    {

        String result = header +
                "> MOCKED\n" ;

        return result;
    }

    /** Log consolidated samples to the dashboard **/
    public String                       logConfigurationHTML()
    {
        String result = "<p> MOCKED </p>" ;

        return result;
    }

}

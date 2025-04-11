/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Into-The-Deep Sample detection processing
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.algorithms.vision;

/* System includes */
import java.util.ArrayList;
import java.util.List;

/* Android includes */
import android.graphics.Bitmap;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Opencv includes */
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.CvType;
import org.opencv.core.Size;
import org.opencv.android.Utils;
import org.opencv.imgproc.Imgproc;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

public class SamplesDetection implements Configurable {

    public enum Mode {
        NONE,
        DETECT,
        ORIENT,
        TRACK }

    final LogManager                mLogger;

    protected boolean               mConfigurationValid;

    final String                    mName;

    Mode                            mMode;
    LimelightObjectDetection        mDetection;
    LimelightObjectOrientation      mOrientation;
    LimelightFixedObjectTracking    mTracking;

    List<Sample>                    mOngoing;
    List<Sample>                    mConsolidated;
    Sample                          mSelected;

    int                             mImageIndex;

    /**
     * Constructor
     *
     * @param name The camera name
     * @param map The hardware map to get sensors from
     * @param logger The logger to use for traces
     */
    public  SamplesDetection(String name, HardwareMap map, LogManager logger) {

        mLogger             = logger;
        mConfigurationValid = false;
        mName               = name;

        mMode               = Mode.NONE;
        mDetection          = new LimelightObjectDetection("detection", map, logger);
        mOrientation        = new LimelightObjectOrientation("orientation", map, logger);
        mTracking           = new LimelightFixedObjectTracking("tracking", map, logger);

        mImageIndex         = 0;
        mOngoing            = new ArrayList<>();
        mConsolidated       = new ArrayList<>();
        mSelected           = null;

    }

    public List<Sample>                     samples() { return mConsolidated; }
    public List<Sample>                     ongoing() { return mOngoing; }

    /**
     * Start sample detection
     */
    public void                             start() {

        if(mConfigurationValid) {
            mMode = Mode.DETECT;
            mDetection.start();
            mImageIndex = 0;
        }

    }

    /**
     * Detect new samples
     */
    public void                             detect() {

        if(mConfigurationValid) {

            if (mMode == Mode.DETECT) {
                List<Sample> detections = mDetection.process();
                mLogger.info(""+detections.size());
                if (!detections.isEmpty()) {
                    mOngoing.addAll(detections);
                    mLogger.info("Switching to orientation");
                    mOrientation.start(mOngoing);
                    mMode = Mode.ORIENT;
                }
            } else if (mMode == Mode.ORIENT) {
                List<Sample> oriented = mOrientation.process();
                if (!oriented.isEmpty()) {
                    mLogger.info("Switching to detection");
                    mConsolidated.clear();
                    mConsolidated.addAll(oriented);
                    // More probable
                    mSelected = mConsolidated.get(0);
                    mDetection.start();
                    mMode = Mode.DETECT;
                    mOngoing.clear();
                }
            } else if (mMode == Mode.TRACK) {
                mMode = Mode.DETECT;
                mDetection.start();
            }

            mLogger.info("Processing image " + mImageIndex);
            mImageIndex++;

        }

    }

    /**
     * Track detected samples
     */
    public void                             track() {

        if(mConfigurationValid) {

            if (mMode != Mode.TRACK) {
                mTracking.start(mImageIndex, mConsolidated);
                mMode = Mode.TRACK;
            } else {
                mConsolidated = mTracking.process();
            }
        }

    }

    /**
     * Create an empty bitmap, black content, to add overlays to
     *
     * @param width : width of the image to create
     * @param height : height of the image to create
     *
     * @return An android bitmap of a black imafge with samples overlays
     * **/
    public Bitmap                           draw(int width, int height) {
        // Create a blank image (black)
        Mat frame = new Mat(height, width, CvType.CV_8UC3, new Scalar(0, 0, 0));

        for(Sample sample : mConsolidated) {

            Point topLeft = new Point(sample.xMin(), sample.yMin());
            Point bottomRight = new Point(sample.xMax(), sample.yMax());

            Scalar color = new Scalar(255,255,255);
            if(sample.color() == Sample.Color.RED) { color = new Scalar(255,0,0); }
            if(sample.color() == Sample.Color.BLUE) { color = new Scalar(0,0,255); }
            if(sample.color() == Sample.Color.YELLOW) { color = new Scalar(255,255,0); }

            Point cross1 = new Point(Math.max(0,sample.x() - 10), sample.y());
            Point cross2 = new Point(Math.min(width - 1,sample.x() + 10), sample.y());
            Point cross3 = new Point(sample.x(), Math.max(0,sample.y() - 10));
            Point cross4 = new Point(sample.x(),Math.min(height - 1,sample.y() + 10));

            String text = sample.index() + " - " +  String.format("%.2f",sample.confidence()) + " - " + (int)(sample.orientation());
            Size size = Imgproc.getTextSize(text, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,1,null);
            Point text1 = new Point(sample.xMin(), Math.max(0,sample.yMin() - size.height));
            Point text2 = new Point(Math.min(width - 1,sample.xMin() + size.width), sample.yMin());

            Imgproc.rectangle(frame, topLeft, bottomRight, color, 2);
            Imgproc.line(frame, cross1, cross2, color, 2);
            Imgproc.line(frame, cross3, cross4, color, 2);
            Imgproc.rectangle(frame, text1, text2, color,Core.FILLED);
            if(sample.index() == mSelected.index()) {
                Imgproc.putText(frame, text, new Point(sample.xMin(), sample.yMin()), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
            }
            else {
                Imgproc.putText(frame, text, new Point(sample.xMin(), sample.yMin()), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 0), 1);
            }
        }

        // Convert to Bitmap to show in Android
        Bitmap result = Bitmap.createBitmap(frame.cols(), frame.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(frame, result);
        return result;
    }

    /** Log consolidated samples to the dashboard **/
    public void                             log()
    {

        StringBuilder result = new StringBuilder();

        result.append("<details style=\"margin-left:10px\">\n");
        result.append("<summary style=\"font-size: 12px; font-weight: 500\"> SAMPLES </summary>\n");
        result.append("<ul>\n");
        for (Sample sample : mConsolidated) {
            result.append(sample.logHTML());
        }
        result.append("</ul>\n");
        result.append("</details>\n");

        mLogger.raw(LogManager.Target.DASHBOARD,result.toString());
    }

    /**
     * Determines if the sample detection is configured correctly.
     *
     * @return True if the algorithm is configured, false otherwise.
     */
    @Override
    public boolean                          isConfigured() { return mConfigurationValid;}

    /**
     * Reads and applies the algorithm configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    public void                             read(JSONObject reader) {

        mConfigurationValid = true;

        try {

            if (reader.has(LimelightObjectDetection.sTypeKey)) {
               mDetection.read(reader.getJSONObject(LimelightObjectDetection.sTypeKey));
            }
            if (reader.has(LimelightObjectOrientation.sTypeKey)) {
                mOrientation.read(reader.getJSONObject(LimelightObjectOrientation.sTypeKey));
            }
            if (reader.has(LimelightFixedObjectTracking.sTypeKey)) {
                mTracking.read(reader.getJSONObject(LimelightFixedObjectTracking.sTypeKey));
            }

            if(!mDetection.isConfigured())      {
                mLogger.error("Detection configuration is not valid");
                mConfigurationValid = false; }
            if(!mOrientation.isConfigured())    {
                mLogger.error("Orientation configuration is not valid");
                mConfigurationValid = false;
            }
            if(!mTracking.isConfigured())       {
                mLogger.error("Tracking configuration is not valid");
                mConfigurationValid = false;
            }

        } catch (JSONException e) {
            mLogger.error(e.getMessage());
        }
    }

    /**
     * Writes the current algorithm configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    public void                         write(JSONObject writer) {

        try {

            // Write detection configuration
            JSONObject detection = new JSONObject();
            mDetection.write(detection);
            writer.put(LimelightObjectDetection.sTypeKey,detection);

            // Write orientation configuration
            JSONObject orientation = new JSONObject();
            mOrientation.write(orientation);
            writer.put(LimelightObjectOrientation.sTypeKey,orientation);

            // Write tracking configuration
            JSONObject tracking = new JSONObject();
            mTracking.write(tracking);
            writer.put(LimelightFixedObjectTracking.sTypeKey,tracking);

        } catch (JSONException e) { mLogger.error(e.getMessage()); }
    }

    public String                       logConfigurationHTML()
    {

        String result = "<details style=\"margin-left:10px\">\n" +
                "<summary style=\"font-size: 12px; font-weight: 500\"> SAMPLE DETECTION </summary>\n" +
                "<ul>\n" +

                // Log pipelines
                mDetection.logConfigurationHTML() +
                mOrientation.logConfigurationHTML() +
                mTracking.logConfigurationHTML() +
                "</ul>\n" +
                "</details>\n";

        return result;

    }

    public String                       logConfigurationText(String header)
    {

        String result = header +
                mDetection.logConfigurationText(header + "--") +
                "\n" +
                header +
                mOrientation.logConfigurationText(header + "--") +
                "\n" +
                header +
                mTracking.logConfigurationText(header + "--") +
                "\n";

        return result;

    }

}

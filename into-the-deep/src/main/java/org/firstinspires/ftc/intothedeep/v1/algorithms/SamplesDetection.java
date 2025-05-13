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
import java.util.Comparator;

/* Android includes */
import android.graphics.Bitmap;

/* Opencv includes */

import org.json.JSONException;
import org.json.JSONObject;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.CvType;
import org.opencv.core.Size;
import org.opencv.android.Utils;
import org.opencv.imgproc.Imgproc;

/* Tools includes */
import org.firstinspires.ftc.core.tools.Calibration;
import org.firstinspires.ftc.core.tools.LogManager;

/* Vision includes */
import org.firstinspires.ftc.core.algorithms.Algorithm;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

public class SamplesDetection implements Algorithm {

    public enum Mode {
        NONE,
        DETECT,
        ORIENT
    }

    final LogManager                mLogger;

    boolean                         mConfigurationValid;
    final String                    mName;

    LimelightObjectDetection        mDetection;
    LimelightObjectOrientation      mOrientation;
    Calibration                     mCalibration;

    Mode                            mMode;
    Sample.Color                    mColor;
    int                             mImageIndex;

    List<Sample>                    mOngoing;
    List<Sample>                    mConsolidated;
    Sample                          mSelected;


    /**
     * Constructor
     *
     * @param name The camera name
     * @param hardware The hardware data to get sensors from
     * @param logger The logger to use for traces
     */
    public  SamplesDetection(String name, Hardware hardware, LogManager logger) {

        mLogger             = logger;

        mName               = name;

        mConfigurationValid = false;

        mDetection          = new LimelightObjectDetection(name, hardware, logger);
        mOrientation        = new LimelightObjectOrientation(name, hardware, logger);
        mCalibration        = new Calibration();

        mImageIndex         = 0;
        mMode               = Mode.NONE;
        mColor              = Sample.Color.UNKNOWN;

        mConsolidated       = new ArrayList<>();
        mOngoing            = new ArrayList<>();
        mSelected           = null;

    }

    public List<Sample>                     samples() { return mConsolidated; }

    /**
     * Start sample detection
     */
    public void                             start() {
        mMode       = Mode.DETECT;
        mImageIndex = 0;
        mDetection.start();
        mCalibration.initialize();
    }

    /**
     * Detect new samples
     */
    public void                             update() {

        if(mMode != Mode.NONE) {
            mLogger.info(LogManager.Target.FILE, mName + " start");
        }
        if (mMode == Mode.DETECT) {

            List<Sample> detections = mDetection.process();
            mLogger.debug(LogManager.Target.FILE," Neural detector found " + detections.size() + " samples");

            if (!detections.isEmpty()) {

                mOngoing.clear();
                for (Sample sample : detections) {
                    float[] ground = mCalibration.computeGroundPosition(sample.x(), sample.y());
                    sample.distanceX(ground[1]);
                    sample.distanceY(-ground[0]);
                    mLogger.debug(sample.logHTML());
                    mOngoing.add(sample);
                }
                mOngoing.sort(Comparator.comparingDouble(s -> mergedRanking(s, mColor)));
                for (Sample sample : mOngoing) {
                    mLogger.debug(sample.logHTML());
                }

//                mLogger.addLine("Switching to orientation");
                mOrientation.start(mOngoing);
                mMode = Mode.ORIENT;
            }
        }
        else if (mMode == Mode.ORIENT) {
            List<Sample> oriented = mOrientation.process();
            mLogger.debug(LogManager.Target.FILE," Orientation processed for " + oriented.size() + " samples");
            if (!oriented.isEmpty()) {

//              mLogger.addLine("Switching back to detection");
                mConsolidated.clear();
                mConsolidated.addAll(oriented);
                mSelected = mConsolidated.get(0);
                mDetection.start();
                mMode = Mode.DETECT;
                mOngoing.clear();
            }
            for (Sample sample : mConsolidated) {
                mLogger.debug(sample.logHTML());
            }
            mLogger.debug(mSelected.logHTML());
        }

        mImageIndex++;
    }

    private static double distanceRanking(Sample s, double x0, double y0) {
        double dx = s.x() - x0;
        double dy = s.y() - y0;
        return dx * dx + dy * dy;
    }

    private static double confidenceRanking(Sample s) {
        return s.confidence();
    }

    private static double mergedRankingOnClaw(Sample s, Sample.Color color) {
        double result = 10000;
        if(s.color() == color || color == Sample.Color.UNKNOWN) {
            result = s.distanceY();
        }
        return result;
    }

    private static double mergedRanking(Sample s, Sample.Color color) {
        double result = 10000;
        if(s.color() == color || color == Sample.Color.UNKNOWN) {
            result = Math.sqrt(s.distanceX()*s.distanceX()+s.distanceY()*s.distanceY());
        }
        return result;
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


    public void                             read(JSONObject reader) {

        mConfigurationValid = true;

        try {

            if (reader.has(LimelightObjectDetection.sTypeKey)) {
                JSONObject detection = reader.getJSONObject(LimelightObjectDetection.sTypeKey);
                mDetection.read(detection);
            }

            if (reader.has(LimelightObjectOrientation.sTypeKey)) {

                JSONObject detection = reader.getJSONObject(LimelightObjectOrientation.sTypeKey);
                mOrientation.read(detection);
            }

            if(!mDetection.isConfigured()) {
                mLogger.warning("Detection configuration is invalid");
                mConfigurationValid = false;
            }
            if(!mOrientation.isConfigured()) {
                mLogger.warning("Orientation configuration is invalid");
                mConfigurationValid = false;
            }

        } catch (JSONException e) {
            mLogger.error(e.getMessage());
        }
    }

    public void                         write(JSONObject writer) {

        try {

            // Write detection
            JSONObject detection = new JSONObject();
            mDetection.write(detection);
            writer.put(LimelightObjectDetection.sTypeKey,detection);

            // Write orientation
            JSONObject orientation = new JSONObject();
            mOrientation.write(orientation);
            writer.put(LimelightObjectOrientation.sTypeKey,orientation);

        } catch (JSONException e) { mLogger.error(e.getMessage()); }
    }


    /** Log consolidated samples to the dashboard **/
    public void                             log()
    {

        StringBuilder result = new StringBuilder();

        result.append("<details open style=\"margin-left:10px\">\n");
        result.append("<summary style=\"font-size: 12px; font-weight: 500\"> ONGOING </summary>\n");
        result.append("<ul>\n");
        for (Sample sample : mOngoing) {
            result.append(sample.logHTML());
        }
        result.append("</ul>\n");
        result.append("</details>\n");

        result.append("<details open style=\"margin-left:10px\">\n");
        result.append("<summary style=\"font-size: 12px; font-weight: 500\"> CONSOLIDATED </summary>\n");
        result.append("<ul>\n");
        for (Sample sample : mConsolidated) {
            result.append(sample.logHTML());
        }
        result.append("</ul>\n");
        result.append("</details>\n");

        if (mSelected != null){
            result.append("<div>\n");
            result.append(mSelected.logHTML());
            result.append("</div>\n");
        }
    }

    /** Log consolidated samples to the dashboard **/
    public String                             logConfigurationText(String header)
    {

        String result = header +
                "> SAMPLE DETECTION\n" +
                header +
                "--> DETECTION\n" +
                mDetection.logConfigurationText((header + "----")) +
                header +
                "--> ORIENTATION\n" +
                mOrientation.logConfigurationText((header + "----"));

        return result;
    }

    /** Log consolidated samples to the dashboard **/
    public String                             logConfigurationHTML()
    {


        String result = "<details style=\"margin-left:10px\">\n" +
                "<summary style=\"font-size: 12px; font-weight: 500\"> DETECTION </summary>\n" +
                "<ul>\n" +
                mDetection.logConfigurationHTML() +
                "</ul>\n" +
                "</details>\n" +
                "<details style=\"margin-left:10px\">\n" +
                "<summary style=\"font-size: 12px; font-weight: 500\"> ORIENTATION </summary>\n" +
                "<ul>\n" +
                mOrientation.logConfigurationHTML() +
                "</ul>\n" +
                "</details>\n" +
                "</ul>\n" +
                "</details>\n";

        return result;
    }

}

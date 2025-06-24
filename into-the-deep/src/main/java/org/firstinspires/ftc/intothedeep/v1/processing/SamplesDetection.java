/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Into-The-Deep Sample detection processing
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.processing;

/* System includes */
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.ByteArrayOutputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Comparator;
import java.util.Objects;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Opencv includes */
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/* Tools includes */
import org.firstinspires.ftc.core.tools.Calibration;
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.intothedeep.v1.configuration.LimelightPipelinesCode;

/* Processing includes */
import org.firstinspires.ftc.core.processing.limelight.LimelightNeuralNetworkDetection;
import org.firstinspires.ftc.core.processing.limelight.LimelightPythonSnapscript;
import org.firstinspires.ftc.core.processing.Processor;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

public class SamplesDetection implements Processor {

    static final String         sTypeValue = "sample-detection";


    public enum Mode {
        NONE,
        DETECT,
        ORIENT
    }

    LogManager                                      mLogger;

    boolean                                         mConfigurationValid;
    String                                          mName;

    SampleFactory                                   mFactory;
    LimelightNeuralNetworkDetection<Sample>         mDetection;
    LimelightPythonSnapscript<Sample,Sample>        mOrientation;
    Calibration                                     mCalibration;

    Mode                                            mMode;
    Sample.Color                                    mColor;
    int                                             mImageIndex;

    List<Sample>                                    mOngoing;
    List<Sample>                                    mConsolidated;
    Sample                                          mSelected;

    protected SamplesDetection() {}

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

        byte[]  model       = this.readModel();
        String  labels      = this.readLabels();
        mFactory            = new SampleFactory(mLogger);
        mDetection          = new LimelightNeuralNetworkDetection<>(SampleFactory.sDetectionKey,mFactory, model, labels, hardware, logger);
        mOrientation        = new LimelightPythonSnapscript<>(SampleFactory.sOrientationKey, mFactory,LimelightPipelinesCode.sOrientationCode, hardware, logger);
        mCalibration        = new Calibration();

        mImageIndex         = 0;
        mMode               = Mode.NONE;
        mColor              = Sample.Color.UNKNOWN;

        mConsolidated       = new ArrayList<>();
        mOngoing            = new ArrayList<>();
        mSelected           = null;

    }

    public void                             color(Sample.Color color) { mColor = color; }

    public String                           name() { return mName; }

    public List<Sample>                     samples() { return mConsolidated; }

    public boolean                          isConfigured() { return mConfigurationValid; }

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
                    mOngoing.add(sample);
                }
                mOngoing.sort(Comparator.comparingDouble(s -> mergedRanking(s, mColor)));

                mLogger.debug("Switching to orientation");
                mOrientation.start(mOngoing);
                mMode = Mode.ORIENT;
            }
        }
        else if (mMode == Mode.ORIENT) {
            List<Sample> oriented = mOrientation.process();
            mLogger.debug(LogManager.Target.FILE," Orientation processed for " + oriented.size() + " samples");
            if (!oriented.isEmpty()) {

                for (Sample sample : oriented ) {
                    int index = sample.index();
                    for(Sample ongoing : mOngoing) {
                        if(ongoing.index() == index) {
                            sample.distanceX(ongoing.distanceX());
                            sample.distanceY(ongoing.distanceY());
                        }
                    }

                }

                mLogger.debug("Switching back to detection");
                mConsolidated.clear();
                mConsolidated.addAll(oriented);
                Sample candidate = mConsolidated.get(0);
                if(candidate.color() == mColor || mColor == Sample.Color.UNKNOWN) {
                    mSelected = candidate;
                }
                else {
                    mSelected = null;
                }
                mDetection.start();
                mMode = Mode.DETECT;
                mOngoing.clear();
            }
        }

        mImageIndex++;
    }

    private static double               distanceRanking(Sample s, double x0, double y0) {
        double dx = s.x() - x0;
        double dy = s.y() - y0;
        return dx * dx + dy * dy;
    }

    private static double               confidenceRanking(Sample s) {
        return s.confidence();
    }

    private static double               mergedRankingOnClaw(Sample s, Sample.Color color) {
        double result = 10000;
        if(s.color() == color || color == Sample.Color.UNKNOWN) {
            result = s.distanceY();
        }
        return result;
    }

    private static double               mergedRanking(Sample s, Sample.Color color) {
        double result = 10000;
        if(s.color() == color || color == Sample.Color.UNKNOWN) {
            result = Math.sqrt(s.distanceX()*s.distanceX()+s.distanceY()*s.distanceY());
        }
        return result;
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

            for (Sample sample : mConsolidated) {

                Point topLeft = new Point(sample.xMin(), sample.yMin());
                Point bottomRight = new Point(sample.xMax(), sample.yMax());

                Scalar color = new Scalar(255, 255, 255);
                if (sample.color() == Sample.Color.RED) {
                    color = new Scalar(255, 0, 0);
                }
                if (sample.color() == Sample.Color.BLUE) {
                    color = new Scalar(0, 0, 255);
                }
                if (sample.color() == Sample.Color.YELLOW) {
                    color = new Scalar(255, 255, 0);
                }

                Point cross1 = new Point(Math.max(0, sample.x() - 10), sample.y());
                Point cross2 = new Point(Math.min(raw.cols() - 1, sample.x() + 10), sample.y());
                Point cross3 = new Point(sample.x(), Math.max(0, sample.y() - 10));
                Point cross4 = new Point(sample.x(), Math.min(raw.rows() - 1, sample.y() + 10));

                String text = sample.index() + " - " + String.format("%.2f", sample.confidence()) + " - " + (int) (sample.orientation());
                Size size = Imgproc.getTextSize(text, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, 1, null);
                Point text1 = new Point(sample.xMin(), Math.max(0, sample.yMin() - size.height));
                Point text2 = new Point(Math.min(raw.cols() - 1, sample.xMin() + size.width), sample.yMin());

                Imgproc.rectangle(result, topLeft, bottomRight, color, 2);
                Imgproc.line(result, cross1, cross2, color, 2);
                Imgproc.line(result, cross3, cross4, color, 2);
                Imgproc.rectangle(result, text1, text2, color, Core.FILLED);
                if (mSelected != null && sample.index() == mSelected.index()) {
                    Imgproc.putText(result, text, new Point(sample.xMin(), sample.yMin()), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
                } else {
                    Imgproc.putText(result, text, new Point(sample.xMin(), sample.yMin()), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 0), 1);
                }
            }
        }

        return result;
    }


    public void                             read(JSONObject reader) {

        mConfigurationValid = true;

        try {

            if (reader.has(SampleFactory.sDetectionKey)) {
                JSONObject detection = reader.getJSONObject(SampleFactory.sDetectionKey);
                mDetection.read(detection);
            }

            if (reader.has(SampleFactory.sOrientationKey)) {

                JSONObject detection = reader.getJSONObject(SampleFactory.sOrientationKey);
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
            writer.put(SampleFactory.sDetectionKey,detection);

            // Write orientation
            JSONObject orientation = new JSONObject();
            mOrientation.write(orientation);
            writer.put(SampleFactory.sOrientationKey,orientation);

        } catch (JSONException e) { mLogger.error(e.getMessage()); }
    }


    /** Log sample detection results **/
    public void                         log(String header)
    {

        StringBuilder result = new StringBuilder();

        result.append("<p> Current mode is ")
                .append(mMode)
                .append("</p>");

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
            result.append("<p style=\"margin-left:10px; font-size: 12px; font-weight: 500\"> SELECTED : ");
            result.append(mSelected.logHTML());
            result.append("</p>\n");
        }

        mLogger.raw(LogManager.Target.DASHBOARD, result.toString());

        result = new StringBuilder();

        result.append(header);
        result.append("> ONGOING\n");
        for (Sample sample : mOngoing) {
            result.append(sample.logText(header+"--"));
        }

        result.append(header);
        result.append("> CONSOLIDATED\n");
        for (Sample sample : mConsolidated) {
            result.append(sample.logText(header+"--"));
        }

        if (mSelected != null){result.append(header);
            result.append("> SELECTED\n");
            result.append(mSelected.logText(header+"--"));
        }

        mLogger.info(LogManager.Target.FILE,result.toString());
    }

    /** Log consolidated samples to the dashboard **/
    public String                       logConfigurationText(String header)
    {

        String result = header +
                "> SAMPLE DETECTION\n" +
                header +
                "--> DETECTION\n" +
                mDetection.logConfigurationText((header + "----")) +
                "\n" +
                header +
                "--> ORIENTATION\n" +
                mOrientation.logConfigurationText((header + "----"));

        return result;
    }

    /** Log consolidated samples to the dashboard **/
    public String                       logConfigurationHTML()
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

    byte[]                              readModel()
    {
        byte[] result = null;
        try {
            InputStream modelStream = Objects.requireNonNull(getClass().getClassLoader()).getResourceAsStream(LimelightPipelinesCode.sModelPath);

            ByteArrayOutputStream buffer = new ByteArrayOutputStream();
            byte[] temp = new byte[4096];
            int bytesRead = modelStream.read(temp);
            while (bytesRead != -1) {
                buffer.write(temp, 0, bytesRead);
                bytesRead = modelStream.read(temp);
            }
            result =  buffer.toByteArray();
        }
        catch(IOException e) {
            String filename = getClass().getClassLoader().getResource(LimelightPipelinesCode.sModelPath).getFile();
            mLogger.error("Can't read model file " + filename + " : " + e);
        }

        return result;
    }

    String                              readLabels()
    {
        String result = "";
        try {

            StringBuilder content = new StringBuilder();
            try (InputStream in = Objects.requireNonNull(getClass().getClassLoader()).getResourceAsStream(LimelightPipelinesCode.sLabelsPath);
                 BufferedReader reader = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8))) {

                String line;
                while ((line = reader.readLine()) != null) {
                    content.append(line);
                    content.append('\n');
                }
            }
            result =  content.toString();
        }
        catch(IOException e) {
            String filename = Objects.requireNonNull(getClass().getClassLoader()).getResource(LimelightPipelinesCode.sLabelsPath).getFile();
            mLogger.error("Can't read model file " + filename + " : " + e);
        }

        return result;
    }

}

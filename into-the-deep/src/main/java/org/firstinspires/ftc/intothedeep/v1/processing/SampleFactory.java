package org.firstinspires.ftc.intothedeep.v1.processing;

/* Processing includes */
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.core.processing.limelight.LimelightObjectFactory;
import org.firstinspires.ftc.core.tools.LogManager;

import java.util.List;

public class SampleFactory implements LimelightObjectFactory<Sample, Sample> {

    final static    String  sDetectionKey  = "detection";
    final static    String  sOrientationKey  = "orientation";

    final LogManager    mLogger;

    int                 mSampleId;

    /**
     * Constructor
     *
     * @param logger The logger to use for traces
     */
    public SampleFactory( LogManager logger) {
        mLogger = logger;
        mSampleId = 1;
    }

    /**
     * Build Sample from a neural network detector result
     * @param pipeline Name of the pipeline issuing the sample
     * @param limelight The pipeline result
     * @return A valid Sample to use as interface
     */
    public Sample        build(String pipeline, LLResultTypes.DetectorResult limelight)
    {
        Sample result = null;

        if(pipeline.equals(sDetectionKey)) {

            result = new Sample(mSampleId);

            result.x(limelight.getTargetXPixels());
            result.y(limelight.getTargetYPixels());
            result.area(limelight.getTargetArea());
            result.confidence(limelight.getConfidence());

            if (limelight.getClassName().equals("red")) { result.color(Sample.Color.RED); }
            else if (limelight.getClassName().equals("blue")) { result.color(Sample.Color.BLUE); }
            else if (limelight.getClassName().equals("yellow")) { result.color(Sample.Color.YELLOW); }
            else { result.color(Sample.Color.UNKNOWN); }

            double yMin = 10000;
            double xMax = -1;
            double xMin = 10000;
            double yMax = -1;
            List<List<Double>> corners = limelight.getTargetCorners();
            for (List<Double> corner : corners) {
                if (corner.get(0) < xMin) {
                    xMin = corner.get(0);
                }
                if (corner.get(0) > xMax) {
                    xMax = corner.get(0);
                }
                if (corner.get(1) < yMin) {
                    yMin = corner.get(1);
                }
                if (corner.get(1) > yMax) {
                    yMax = corner.get(1);
                }
            }

            result.xMax(xMax);
            result.xMin(xMin);
            result.yMax(yMax);
            result.yMin(yMin);

            mSampleId ++;

        }
        else { mLogger.warning("Unknown pipeline " + pipeline); }

        return result;
    }

    /**
     * Format sample into a valid input for Limelight python pipeline
     * @param pipeline Name of the pipeline processing the object
     * @param input The object to format
     * @return An llrobot ready array
     */
    public double[]                     format(String pipeline, Sample input) {

        double[] result = null;

        if(pipeline.equals(sOrientationKey)) {

            result = new double[7];
            int i_data = 0;

            Sample.Color color = input.color();
            int col = -1;
            if (color == Sample.Color.RED) { col = 0; }
            else if (color == Sample.Color.BLUE) { col = 1; }
            else if (color == Sample.Color.YELLOW) { col = 2; }

            result[i_data] = input.index(); i_data++;
            result[i_data] = input.x(); i_data++;
            result[i_data] = input.y(); i_data++;
            result[i_data] = input.xMax() - input.xMin(); ; i_data++;
            result[i_data] = input.yMax() - input.yMin(); ; i_data++;
            result[i_data] = col; i_data++;
            result[i_data] = input.area();
        }
        else { mLogger.warning("Unknown pipeline " + pipeline); }

        return result;

    }

    /**
     * return increment from one sample to another in a python snapscript result
     * @param pipeline Name of the pipeline issuing the sample
     * @return The number of data describing an sample
     */
    public int                      increment(String pipeline) {

        int result = 0;

        if(pipeline.equals(sOrientationKey)) {
            result = 10;
        }
        else { mLogger.warning("Unknown pipeline " + pipeline); }

        return result;
    }

    /**
     * Build Sample from a python snapscript result
     * @param pipeline Name of the pipeline issuing the object
     * @param data The pipeline result (llpython)
     * @param position : The position to start reading from
     * @return A valid Sample to use as interface
     */
    public Sample                  build(String pipeline, double[] data, int position)
    {
        Sample result = null;
        if(pipeline.equals(sOrientationKey)) {
            int index = (int) (data[position]);
            if(index != 0) {
                result = new Sample(index);

                result.x(data[position + 1]);
                result.y(data[position + 2]);
                result.orientation(data[position + 4]);
                result.area(data[position + 5]);
                result.xMin(data[position + 6]);
                result.xMax(data[position + 7]);
                result.yMin(data[position + 8]);
                result.yMax(data[position + 9]);

                int col = (int) data[position + 3];

                Sample.Color color = Sample.Color.UNKNOWN;
                if (col == 0) {
                    color = Sample.Color.RED;
                } else if (col == 1) {
                    color = Sample.Color.BLUE;
                } else if (col == 2) {
                    color = Sample.Color.YELLOW;
                }
                result.color(color);

            }
        }
        else { mLogger.warning("Unknown pipeline " + pipeline); }

        return result;


    }

}

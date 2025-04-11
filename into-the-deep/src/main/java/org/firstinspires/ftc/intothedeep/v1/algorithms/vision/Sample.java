/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Limelight object management
   ------------------------------------------------------- */
package org.firstinspires.ftc.intothedeep.v1.algorithms.vision;

/* System includes */
import java.util.List;

/* Qualcomm includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

public class Sample {

    public enum Color {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    };

    int         mIndex;
    double      mX;
    double      mY;
    double      mArea;
    Color       mColor;
    double      mOrientation;
    double      mXMin;
    double      mXMax;
    double      mYMin;
    double      mYMax;
    double      mConfidence;

    LogManager  mLogger;


    /**
     * Constructor
     *
     * @param limelight The camera name
     */
    public  Sample(int index, LLResultTypes.DetectorResult limelight, LogManager logger) {

        mLogger = logger;

        mIndex = index;

        mX = limelight.getTargetXPixels();
        mY = limelight.getTargetYPixels();
        mArea = limelight.getTargetArea();
        mConfidence = limelight.getConfidence();

        if (limelight.getClassName().equals("red")) { mColor = Color.RED; }
        else if (limelight.getClassName().equals("blue")) { mColor = Color.BLUE; }
        else if (limelight.getClassName().equals("yellow")) { mColor = Color.YELLOW; }
        else { mColor = Color.UNKNOWN; }

        mOrientation = 10000;

        mYMin = 10000;
        mXMax = -1;
        mXMin = 10000;
        mYMax = -1;
        List<List<Double>> corners = limelight.getTargetCorners();
        for( List<Double> corner : corners) {
            if (corner.get(0) < mXMin) { mXMin = corner.get(0); }
            if (corner.get(0) > mXMax) { mXMax = corner.get(0); }
            if (corner.get(1) < mYMin) { mYMin = corner.get(1); }
            if (corner.get(1) > mYMax) { mYMax = corner.get(1); }
        }

    }
    public String logHTML() {
        StringBuilder result = new StringBuilder();

        String color = "Unknown";
        if(mColor == Color.YELLOW) { color = "yellow"; }
        if(mColor == Color.RED) { color = "red"; }
        if(mColor == Color.BLUE) { color = "blue"; }

        result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                .append("X : ")
                .append(mX)
                .append(" - Y : ")
                .append(mY)
                .append(" - COLOR : ")
                .append(color)
                .append(" - AREA : ")
                .append(mArea)
                .append(" - ORIENTATION : ")
                .append(mOrientation)
                .append(" - BBOX : ")
                .append(mXMin)
                .append(":")
                .append(mXMax)
                .append(",")
                .append(mYMin)
                .append(":")
                .append(mYMax)
                .append("</li>\n");

        return result.toString();
    }

    public int    index()       { return mIndex; }

    public double x()           { return mX; }

    public double y()           { return mY; }
    public double area()        { return mArea; }
    public Color  color()       { return mColor; }
    public double orientation() { return mOrientation; }
    public double xMax()        { return mXMax; }
    public double xMin()        { return mXMin; }
    public double yMin()        { return mYMin; }
    public double yMax()        { return mYMax; }
    public double confidence()  { return mConfidence; }


    public void     x(double value)           { mX = value; }
    public void     y(double value)           { mY = value; }
    public void     orientation(double value) { mOrientation = value; }
    public void     color(Color value)        { mColor = value; }
    public void     xMin(double value)        { mXMin = value; }
    public void     xMax(double value)        { mXMax = value; }
    public void     yMin(double value)        { mYMin = value; }
    public void     yMax(double value)        { mYMax = value; }



}

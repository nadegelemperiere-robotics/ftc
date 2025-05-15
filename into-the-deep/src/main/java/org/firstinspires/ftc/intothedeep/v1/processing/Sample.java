/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Into the deep sample descriptor object
   ------------------------------------------------------- */
package org.firstinspires.ftc.intothedeep.v1.processing;

/* Processing includes */
import org.firstinspires.ftc.core.processing.limelight.LimelightObject;

public class Sample implements LimelightObject {

    public enum Color {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    static final double sInvalidOrientation = -10000000000.0;

    final int   mIndex;
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

    double      mDistanceX;
    double      mDistanceY;


    /**
     * Constructor
     *
     * @param index The sample index
     */
    public Sample(int index) {
        mIndex = index;
        mOrientation = sInvalidOrientation;
    }

    public String logHTML() {
        StringBuilder result = new StringBuilder();

        String color = "Unknown";
        if (mColor == Color.YELLOW) { color = "yellow"; }
        if (mColor == Color.RED) { color = "red"; }
        if (mColor == Color.BLUE) { color = "blue"; }

        result.append("<li style=\"padding-left:10px; font-size: 11px\">")
                .append("INDEX : ")
                .append(mIndex)
                .append(" - X : ")
                .append(mX)
                .append(" - Y : ")
                .append(mY)
                .append(" - COLOR : ")
                .append(color)
                .append(" - AREA : ")
                .append(mArea)
                .append(" - ORIENTATION : ")
                .append(this.orientation())
                .append(" - DISTANCE X : ")
                .append(mDistanceX)
                .append(" - Y : ")
                .append(mDistanceY)
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

    public String logText(String header) {

        StringBuilder result = new StringBuilder();

        String color = "Unknown";
        if (mColor == Color.YELLOW) { color = "yellow"; }
        if (mColor == Color.RED) { color = "red"; }
        if (mColor == Color.BLUE) { color = "blue"; }

        result.append(header)
                .append("> INDEX : ")
                .append(mIndex)
                .append(" - X : ")
                .append(mX)
                .append(" - Y : ")
                .append(mY)
                .append(" - COLOR : ")
                .append(color)
                .append(" - AREA : ")
                .append(mArea)
                .append(" - ORIENTATION : ")
                .append(this.orientation())
                .append(" - DISTANCE X : ")
                .append(mDistanceX)
                .append(" - Y : ")
                .append(mDistanceY)
                .append(" - BBOX : ")
                .append(mXMin)
                .append(":")
                .append(mXMax)
                .append(",")
                .append(mYMin)
                .append(":")
                .append(mYMax)
                .append("\n");

        return result.toString();
    }

    public int    index()       { return mIndex;  }

    public double x()           { return mX;  }
    public double y()           { return mY;  }
    public double area()        { return mArea;   }
    public Color  color()       { return mColor;  }
    public double xMax()        { return mXMax; }
    public double xMin()        { return mXMin; }
    public double yMin()        { return mYMin; }
    public double yMax()        { return mYMax; }
    public double confidence()  { return mConfidence; }
    public double distanceX()   { return mDistanceX; }
    public double distanceY()   { return mDistanceY; }


    public double orientation() {

        double result;
        if(mOrientation != sInvalidOrientation) { result = mOrientation; }
        else {
            double width = mXMax - mXMin;
            double height = mYMax - mYMin;
            double ratio = height / width;

            if(ratio != 2.33) {
                double tan_angle = (ratio * 2.33 - 1) / (2.33 - ratio);
                result = Math.toDegrees(Math.atan(tan_angle));
            }
            else {
                result = 90;
            }
        }

        return result;
    }


    public void x(double value)             { mX = value; }
    public void y(double value)             { mY = value; }
    public void orientation(double value)   { mOrientation = value; }
    public void area(double value)          { mArea = value; }
    public void color(Color value)          { mColor = value; }
    public void distanceX(double value)     { mDistanceX = value; }
    public void distanceY(double value)     { mDistanceY = value; }
    public void xMin(double value)          { mXMin = value; }
    public void xMax(double value)          { mXMax = value; }
    public void yMin(double value)          { mYMin = value; }
    public void yMax(double value)          { mYMax = value; }

    public void confidence(double value)    {mConfidence = value; }

}
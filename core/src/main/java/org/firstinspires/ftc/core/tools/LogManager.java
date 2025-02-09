/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Logging manager
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.tools;

/* System includes */
import java.util.Map;
import java.util.LinkedHashMap;
import java.util.Objects;
import java.util.logging.FileHandler;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;
import java.util.logging.Level;
import java.io.IOException;

/* Android includes */
import android.os.Environment;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* ACME robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;

public class LogManager{

    public enum Target {
        DRIVER_STATION,
        DASHBOARD,
        FILE
    }

    // Formatting
    static  final   int         sErrorFontSize   = 14;
    static  final   int         sWarningFontSize = 14;
    static  final   int         sMetricsFontSize = 13;
    static  final   int         sEntryFontSize   = 15;

    // Loggers
    Telemetry                   mDriverStation;
    FtcDashboard                mDashboard;
    Logger                      mFile;

    // Persistence
    Map<Target,StringBuilder>       mErrors;
    Map<Target,StringBuilder>       mWarnings;
    Map<Target,StringBuilder>       mInfos;
    Map<Target,StringBuilder>       mDebugs;
    Map<Target,StringBuilder>       mTraces;
    Map<Target,Map<String,String>>  mMetrics;

    /**
     * Builds a log manager
     *
     * @param station the driver station telemetry from opmode (may be null if shall not be used for logging)
     * @param dashboard the FTC dashboard instance (may be null if shall not be used for logging)
     * @param filename the name of the file to log into (may be empty if shall not be used for logging)
     */
    public LogManager(Telemetry station, FtcDashboard dashboard, String filename) {

        mDriverStation = station;
        if(mDriverStation != null) {
            mDriverStation.setAutoClear(false);
        }

        mDashboard = dashboard;
        if(mDashboard != null) {
            mDashboard.getTelemetry().log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        }

        mFile = null;
        String filepath = Environment.getExternalStorageDirectory().getPath()
                + "/FIRST/"
                + filename
                + ".log";
        try {
            if(!filename.isEmpty()) {

                mFile = Logger.getLogger("log-manager");
                FileHandler fileHandler = new FileHandler(filepath,100000,2, true); // Append mode
                fileHandler.setFormatter(new SimpleFormatter());
                mFile.setLevel(Level.ALL);
                mFile.addHandler(fileHandler);
                mFile.setUseParentHandlers(false);
            }
        }
        catch(IOException e)
        {
            mDriverStation.addData("WRN","Unable to open log file " + filepath);
            if(mDashboard != null) {
                mDashboard.getTelemetry().addLine("<p style=\"color: red; font-size: 14px\"> WARNING : Unable to open log file " + filepath + " with exception " + e.getMessage());
            }
        }

        mErrors   = new LinkedHashMap<>();
        mWarnings = new LinkedHashMap<>();
        mInfos    = new LinkedHashMap<>();
        mDebugs   = new LinkedHashMap<>();
        mTraces   = new LinkedHashMap<>();
        mMetrics  = new LinkedHashMap<>();
        for(Target target : Target.values()) {
            mErrors.put(target,new StringBuilder());
            mWarnings.put(target,new StringBuilder());
            mInfos.put(target,new StringBuilder());
            mDebugs.put(target,new StringBuilder());
            mTraces.put(target,new StringBuilder());
            mMetrics.put(target, new LinkedHashMap<>());
        }
    }


    /**
     * Add an error to a specific log sink
     *
     * @param target the log sink
     * @param message the error message
     */
    public void error(Target target, String message) {
        this.error(target,message,4);
    }

    /**
     * Add an error to all log sinks
     *
     * @param message the error message
     */
    public void error(String message) {
        for(Target target : Target.values()) {
            this.error(target,message,4);
        }
    }

    /**
     * Add a warning to a specific log sink
     *
     * @param target the log sink
     * @param message the warning message
     */
    public void warning(Target target, String message) {
        this.warning(target,message,4);
    }

    /**
     * Add a warning to all log sinks
     *
     * @param message the warning message
     */
    public void warning(String message) {
        for(Target target : Target.values()) {
            this.warning(target,message,4);
        }
    }

    /**
     * Add a metric to a specific log sink
     *
     * @param target the log sink
     * @param metric the metric topic
     * @param value the metric value
     */
    public void metric(Target target, String metric, String value) {
        this.metric(target, metric, value, 4);
    }

    /**
     * Add a metric to all log sinks
     *
     * @param metric the metric topic
     * @param value the metric value
     */
    public void metric(String metric, String value) {
        for(Target target : Target.values()) {
            this.metric(target,metric,value,4);
        }
    }

    /**
     * Write current log text to a specific log sink
     *
     * @param target the target log sink
     */
    public void write(Target target) {
        if (target == Target.DRIVER_STATION && mDriverStation != null) {
            mDriverStation.addLine("---------- ERRORS ----------");
            mDriverStation.addLine(Objects.requireNonNull(mErrors.get(Target.DRIVER_STATION)).toString());
            mDriverStation.addLine("--------- WARNINGS ---------");
            mDriverStation.addLine(Objects.requireNonNull(mWarnings.get(Target.DRIVER_STATION)).toString());
            mDriverStation.addLine("--------- METRICS ---------");
            for (Map.Entry<String, String> metric : Objects.requireNonNull(mMetrics.get(target)).entrySet()) {
                mDriverStation.addLine(metric.getKey() + " : " + metric.getValue());
            }

        } else if (target == Target.DASHBOARD && mDashboard != null) {
            StringBuilder persistent = new StringBuilder();

            persistent.append("<details open>\n");
            persistent.append("<summary style=\"font-size:");
            persistent.append(sEntryFontSize);
            persistent.append("px; font-weight: 500\"> ERRORS </summary>\n");
            persistent.append("<ul>\n");
            persistent.append(Objects.requireNonNull(mErrors.get(Target.DASHBOARD)).toString());
            persistent.append("</ul>\n");
            persistent.append("</details>\n");

            persistent.append("<details open>\n");
            persistent.append("<summary style=\"font-size:");
            persistent.append(sEntryFontSize);
            persistent.append("px; font-weight: 500\"> WARNINGS </summary>\n");
            persistent.append("<ul>\n");
            persistent.append(Objects.requireNonNull(mWarnings.get(Target.DASHBOARD)).toString());
            persistent.append("</ul>\n");
            persistent.append("</details>\n");

            persistent.append("<details open>\n");
            persistent.append("<summary style=\"font-size:");
            persistent.append(sEntryFontSize);
            persistent.append("px; font-weight: 500\"> METRICS </summary>\n");
            persistent.append("<ul>\n");
            for (Map.Entry<String, String> metric : Objects.requireNonNull(mMetrics.get(target)).entrySet()) {
                persistent.append("<li style=\"color: black; margin-left:30px; list-style-type: square; font-size: ")
                        .append(sMetricsFontSize)
                        .append("px\">")
                        .append(metric.getKey())
                        .append(" : ")
                        .append(metric.getValue())
                        .append("</li>")
                        .append("\n");
            }
            persistent.append("</ul>\n");
            persistent.append("</details>\n");

            mDashboard.getTelemetry().addLine(persistent.toString());
        }
    }

    /**
     * Write logs accumulated by a specific log sink
     *
     * @param target the target log sink
     */
    public void update(Target target) {
        if (target == Target.DRIVER_STATION && mDriverStation != null) {
            this.write(target);
            mDriverStation.update();
        } else if (target == Target.DASHBOARD && mDashboard != null) {
            this.write(target);
            mDashboard.getTelemetry().update();
        }
    }

    /**
     * Write logs accumulated by one log sink
     */
    public void update() {
        for(Target target : Target.values()) {
            this.update(target);
        }
    }

    /**
     * Clear all logs for a specific sink - persisted ones are not lost
     *
     * @param target the target log sink
     */
    public void clear(Target target) {
        if (target == Target.DRIVER_STATION && mDriverStation != null) {
            mDriverStation.clear();
        } else if (target == Target.DASHBOARD && mDashboard != null) {
            mDashboard.getTelemetry().clear();
        }
    }

    /**
     * Clear logs for all sinks - persisted ones are not lost
     */
    public void clear() {
        for(Target target : Target.values()) {
            this.clear(target);
        }
    }

    /**
     * Clear logs for all sinks - persisted ones are not lost
     */
    public void stop() {
        for(Target target : Target.values()) {
            this.write(target);
        }
    }



    /**
     * Add an error to a log sink
     *
     * @param target the log sink
     * @param message the error description
     * @param source the stack level from which the error was issued
     */
    private void error(Target target, String message, int source) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[source]; // Get caller
        switch(target) {
            case DASHBOARD:
                Objects.requireNonNull(mErrors.get(target))
                        .append("<li style=\"color: red; margin-left:30px; list-style-type: square; font-size: ")
                        .append(sErrorFontSize)
                        .append("px\">")
                        .append(element.getFileName())
                        .append(":")
                        .append(element.getLineNumber())
                        .append(" - error - ")
                        .append(message)
                        .append("</li>")
                        .append("\n");
                break;
            case DRIVER_STATION:
                Objects.requireNonNull(mErrors.get(target))
                        .append(element.getFileName())
                        .append(":")
                        .append(element.getLineNumber())
                        .append(" - error - ")
                        .append(message)
                        .append("\n");
                break;
            case FILE :
                if(mFile != null) {
                    String local = element.getFileName() +
                            ":" +
                            element.getLineNumber() +
                            " - error - " +
                            message +
                            "\n";
                    mFile.log(Level.SEVERE, local);
                }
                break;
        }

    }

    /**
     * Add a warning to a log sink
     *
     * @param target the log sink
     * @param message the error description
     * @param source the stack level from which the warning was issued
     */
    private void warning(Target target, String message, int source) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[source]; // Get caller
        switch(target) {
            case DASHBOARD:
                Objects.requireNonNull(mWarnings.get(target))
                        .append("<li style=\"color: orange; margin-left:30px; list-style-type: square; font-size: ")
                        .append(sWarningFontSize)
                        .append("px\">")
                        .append(element.getFileName())
                        .append(":")
                        .append(element.getLineNumber())
                        .append(" - ")
                        .append(message)
                        .append("</li>")
                        .append("\n");
                break;
            case DRIVER_STATION:
                Objects.requireNonNull(mWarnings.get(target))
                        .append(element.getFileName())
                        .append(":")
                        .append(element.getLineNumber())
                        .append(" - warn - ")
                        .append(message)
                        .append("\n");
                break;
            case FILE :
                if(mFile != null) {
                    String local = element.getFileName() +
                            ":" +
                            element.getLineNumber() +
                            " - " +
                            message +
                            "\n";
                    mFile.log(Level.WARNING, local);
                }
                break;
        }

    }

    /**
     * Add a metric to a log sink
     *
     * @param target the log sink
     * @param metric the metric topic
     * @param value the metric value
     * @param source the stack level from which the metric was issued
     */
    private void metric(Target target, String metric, String value, int source) {
        StackTraceElement element = Thread.currentThread().getStackTrace()[source]; // Get caller
        switch(target) {
            case DASHBOARD:
                Objects.requireNonNull(mMetrics.get(target)).put(metric,value);
                if(mDashboard != null) {
                    Telemetry.Item data = mDashboard.getTelemetry().addData(metric, value);
                    //mDashboard.getTelemetry().removeItem(data);
                }
                break;
            case DRIVER_STATION:
                Objects.requireNonNull(mMetrics.get(target)).put(metric,value);
                break;
            case FILE :
                if(mFile != null) {
                    String local = element.getFileName() +
                            ":" +
                            element.getLineNumber() +
                            " - metric - " +
                            metric + " : " + value +
                            "\n";
                    mFile.log(Level.INFO, local);
                }
                break;
        }
    }
}
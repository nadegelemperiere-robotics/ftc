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
import java.util.Date;
import java.util.logging.FileHandler;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.io.IOException;
import java.text.SimpleDateFormat;

/* Android includes */
import android.os.Environment;

/* Json includes */
import org.json.JSONException;
import org.json.JSONObject;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* ACME robotics includes */
import com.acmerobotics.dashboard.FtcDashboard;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

public class LogManager implements Configurable {

    static final int  sStackLevel = 4;

    public enum Target {
        DRIVER_STATION,
        DASHBOARD,
        FILE
    }

    public enum Severity  {
        ERROR,
        WARNING,
        METRIC,
        INFO,
        DEBUG,
        TRACE
    }

    private static final Map<String, Severity > sConfToLevel = Map.of(
            "error",    Severity.ERROR,
            "warning",  Severity.WARNING,
            "metric",   Severity.METRIC,
            "info",     Severity.INFO,
            "debug",    Severity.DEBUG,
            "trace",    Severity.TRACE
    );

    private static final Map<Severity , String > sLevelToConf = Map.of(
            Severity.ERROR,    "error",
            Severity.WARNING,  "warning",
            Severity.METRIC,   "metric",
            Severity.INFO,     "info",
            Severity.DEBUG,    "debug",
            Severity.TRACE,    "trace"
    );

    private static final Map<Severity , Integer> sLevelToPriority = Map.of(
            Severity.ERROR,    0,
            Severity.WARNING,  1,
            Severity.METRIC,   2,
            Severity.INFO,     3,
            Severity.DEBUG,    4,
            Severity.TRACE,    5
    );

    // Json keys
    static  final   String          sFilenameKey       = "filename";
    static  final   String          sDriverStationKey  = "driver-station";
    static  final   String          sDashboardKey      = "dashboard";
    static  final   String          sLevelKey          = "level";

    // Formatting
    static  final   int             sErrorFontSize   = 14;
    static  final   int             sWarningFontSize = 14;
    public static  final   int      sMetricFontSize  = 13;
    static  final   int             sInfoFontSize    = 13;
    static  final   int             sEntryFontSize   = 15;

    // Status
    boolean                         mConfigurationValid;
    Severity                        mLevel;
    
    // Loggers
    Telemetry                       mDriverStation;
    FtcDashboard                    mDashboard;
    Logger                          mFile;
    String                          mFilename;

    // Persistence
    Map<Target,StringBuilder>       mErrors;
    Map<Target,StringBuilder>       mWarnings;
    Map<Target,Map<String,String>>  mMetrics;

    // Temporary
    Map<Target,StringBuilder>       mInfos;
    Map<Target,StringBuilder>       mDebugs;
    Map<Target,StringBuilder>       mTraces;

    /**
     * Builds a log manager from parameters
     *
     * @param station the driver station telemetry from opmode (may be null if shall not be used for logging)
     * @param dashboard the FTC dashboard instance (may be null if shall not be used for logging)
     * @param filename the name of the file to log into (may be empty if shall not be used for logging)
     */
    public LogManager(Telemetry station, FtcDashboard dashboard, String filename) {
        
        mConfigurationValid = true;
        mLevel = Severity.TRACE;

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

        mDriverStation = station;
        if(mDriverStation != null) {
            mDriverStation.setAutoClear(true);
        }

        mDashboard = dashboard;
        if(mDashboard != null) {
            mDashboard.getTelemetry().log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        }

        mFile = null;
        mFilename = filename;
        try {
            mFile = this.initializeFileLogger(mFilename);
        }
        catch(IOException e)
        {
            mFile = null;
            mConfigurationValid = false;
            this.warning("Unable to open log file " + Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + mFilename + ".log");
        }


    }

    /**
     * Minimal level setter
     *
     * @param level minimal severity to log
     */
    public void level(Severity level) { mLevel = level; }

    /**
     * Configuration checking
     *
     * @return true if object is correctly configured, false otherwise
     */
    public boolean isConfigured() {
        return mConfigurationValid;
    }

    /**
     * Configuration logging into HTML
     *
     * @return configuration as html string
     */
    public String  logConfigurationHTML() {

        String result = "<li style=\"padding-left:10px;font-size:" +
                sMetricFontSize +
                "px\"> " +
                sDriverStationKey +
                ((mDriverStation == null) ? " : false" : " : true") +
                "</li>" +
                "<li style=\"padding-left:10px;font-size:" +
                sMetricFontSize +
                "px\"> " +
                sDashboardKey +
                ((mDashboard == null) ? " : false" : " : true") +
                "</li>" +
                "<li style=\"padding-left:10px;font-size:" +
                sMetricFontSize +
                "px\"> " +
                sFilenameKey + " : " +
                ((mFile == null) ? "" : mFilename) +
                "</li>" +
                "<li style=\"padding-left:10px;font-size:" +
                sMetricFontSize +
                "px\"> " +
                sLevelKey + " : " +
                sLevelToConf.get(mLevel) +
                "</li>";

        return result;
    }
    /**
     * Configuration logging into text
     *
     * @return configuration as basic string
     */
    public String  logConfigurationText(String header) {

        String result = header +
                "> " +
                sDriverStationKey +
                ((mDriverStation == null) ? " : false" : " : true") +
                "\n" +
                header +
                "> " +
                sDashboardKey +
                ((mDashboard == null) ? " : false" : " : true") +
                "\n" +
                header +
                "> " +
                sFilenameKey + " : " +
                ((mFile == null) ? "" : mFilename) +
                "\n" +
                header +
                "> " +
                sLevelKey + " : " +
                sLevelToConf.get(mLevel) +
                "\n";

        return result;
    }

    /**
     * Reads log manager configuration
     *
     * @param reader : JSON object containing configuration
     */
    public void read(JSONObject reader) {

        mConfigurationValid = true;

        if(reader.has(sFilenameKey)) {
            try {
                boolean shallUseFile = reader.getBoolean(sFilenameKey);
                if(shallUseFile && mFile == null) {
                    this.warning("File not provided so can't log to file");
                    mConfigurationValid = false;
                }
                if(!shallUseFile) { mFile = null; }
            }
            catch(JSONException  e) {
                this.error("Error in file logging configuration");
                mFile = null;
                mConfigurationValid = false;
            }
        }
        else { mFile = null; }

        if(reader.has(sDriverStationKey)) {
            try {
                boolean shallUseStation = reader.getBoolean(sDriverStationKey);
                if(shallUseStation && mDriverStation == null) {
                    this.warning("Telemetry not provided so can't log to driver station");
                    mConfigurationValid = false;
                }
                if(!shallUseStation) { mDriverStation = null; }
            }
            catch(JSONException e) {
                this.error("Error in driver station logging configuration");
                mDriverStation = null;
                mConfigurationValid = false;
            }
        }
        else { mDriverStation = null; }

        if(reader.has(sDashboardKey)) {
            try {
                boolean shallUseDashboard = reader.getBoolean(sDashboardKey);
                if(shallUseDashboard && mDashboard == null) {
                    this.warning("Dashboard not provided so can't log to dashboard");
                    mConfigurationValid = false;
                }
                if(!shallUseDashboard) { mDashboard = null; }
            }
            catch(JSONException e) {
                this.error("Error in dashboard logging configuration");
                mDashboard = null;
                mConfigurationValid = false;
            }
        }
        else { mDashboard = null; }

        if(reader.has(sLevelKey)) {
            try {
                String level = reader.getString(sLevelKey);
                if(sConfToLevel.containsKey(level)) {
                    mLevel = sConfToLevel.get(level);
                }
                else { this.warning("Level " + level + " is not managed"); }
            }
            catch(JSONException e) {
                this.error("Error in dashboard logging configuration");
                mDashboard = null;
                mConfigurationValid = false;
            }
        }

    }

    /**
     * Writes log manager configuration
     *
     * @param writer : JSON object to store configuration
     */
    public void write(JSONObject writer) {

        try {
            if(mConfigurationValid) {
                if (mFile != null) {
                    writer.put(sFilenameKey, mFilename);
                }
                if (mDriverStation != null) {
                    writer.put(sDriverStationKey, true);
                }
                if (mDashboard != null) {
                    writer.put(sDashboardKey, true);
                }
            }
        }
        catch(JSONException e ) { this.error("Error writing configuration"); }
    }

    /**
     * Add an error to a specific log sink
     *
     * @param target the log sink
     * @param message the error message
     */
    public void error(Target target, String message) {
        this.error(target, message, sStackLevel);
    }

    /**
     * Add an error to all log sinks
     *
     * @param message the error message
     */
    public void error(String message) {
        for(Target target : Target.values()) {
            this.error(target, message, sStackLevel);
        }
    }

    /**
     * Add a warning to a specific log sink
     *
     * @param target the log sink
     * @param message the warning message
     */
    public void warning(Target target, String message) {
        this.warning(target, message, sStackLevel);
    }

    /**
     * Add a warning to all log sinks
     *
     * @param message the warning message
     */
    public void warning(String message) {
        for(Target target : Target.values()) {
            this.warning(target, message, sStackLevel);
        }
    }

    /**
     * Add a metric to all log sinks
     *
     * @param metric the metric topic
     * @param value the metric value
     */
    public void metric(String metric, String value) {
        for(Target target : Target.values()) {
            this.metric(target, metric, value, sStackLevel);
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
        this.metric(target, metric, value, sStackLevel);
    }

    /**
     * Add an info to all log sinks
     *
     * @param message the info message
     */
    public void info(String message) {
        for(Target target : Target.values()) {
            this.info(target, message, sStackLevel);
        }
    }

    /**
     * Add an info to a specific log sink
     *
     * @param target the log sink
     * @param message the info message
     */
    public void info(Target target, String message) {
        this.info(target, message, sStackLevel);
    }

    /**
     * Add a debug to all log sinks
     *
     * @param message the debug message
     */
    public void debug(String message) {
        for(Target target : Target.values()) {
            this.debug(target, message, sStackLevel);
        }
    }

    /**
     * Add a debug to a specific log sink
     *
     * @param target the log sink
     * @param message the debug message
     */
    public void debug(Target target, String message) {
        this.debug(target, message, sStackLevel);
    }

    /**
     * Add a trace to all log sinks
     *
     * @param message the trace message
     */
    public void trace(String message) {
        for(Target target : Target.values()) {
            this.trace(target, message, sStackLevel);
        }
    }

    /**
     * Add a trace to a specific log sink
     *
     * @param target the log sink
     * @param message the trace message
     */
    public void trace(Target target, String message) {
        this.trace(target, message, sStackLevel);
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
            mDriverStation.addLine("---------- METRICS ---------");
            for (Map.Entry<String, String> metric : Objects.requireNonNull(mMetrics.get(target)).entrySet()) {
                mDriverStation.addLine(metric.getKey() + " : " + metric.getValue());
            }
            mDriverStation.addLine("---------- INFOS ----------");
            mDriverStation.addLine(Objects.requireNonNull(mInfos.get(Target.DRIVER_STATION)).toString());
            mDriverStation.addLine("---------- DEBUGS ---------");
            mDriverStation.addLine(Objects.requireNonNull(mDebugs.get(Target.DRIVER_STATION)).toString());
            mDriverStation.addLine("---------- TRACES ----------");
            mDriverStation.addLine(Objects.requireNonNull(mTraces.get(Target.DRIVER_STATION)).toString());


        } else if (target == Target.DASHBOARD && mDashboard != null) {
            StringBuilder persistent = new StringBuilder();

            persistent.append("<details open>\n");
            persistent.append("<summary style=\"font-size:");
            persistent.append(sEntryFontSize);
            persistent.append("px; font-weight: 500\"> ERRORS </summary>\n");
            persistent.append("<ul>\n");
            persistent.append(Objects.requireNonNull(mErrors.get(Target.DASHBOARD)));
            persistent.append("</ul>\n");
            persistent.append("</details>\n");

            persistent.append("<details open>\n");
            persistent.append("<summary style=\"font-size:");
            persistent.append(sEntryFontSize);
            persistent.append("px; font-weight: 500\"> WARNINGS </summary>\n");
            persistent.append("<ul>\n");
            persistent.append(Objects.requireNonNull(mWarnings.get(Target.DASHBOARD)));
            persistent.append("</ul>\n");
            persistent.append("</details>\n");

            persistent.append("<details open>\n");
            persistent.append("<summary style=\"font-size:");
            persistent.append(sEntryFontSize);
            persistent.append("px; font-weight: 500\"> METRICS </summary>\n");
            persistent.append("<ul>\n");
            for (Map.Entry<String, String> metric : Objects.requireNonNull(mMetrics.get(target)).entrySet()) {
                persistent.append("<li style=\"color: black; margin-left:30px; list-style-type: square; font-size: ")
                        .append(sMetricFontSize)
                        .append("px\">")
                        .append(metric.getKey())
                        .append(" : ")
                        .append(metric.getValue())
                        .append("</li>")
                        .append("\n");
            }
            persistent.append("</ul>\n");
            persistent.append("</details>\n");

            persistent.append("<details open>\n");
            persistent.append("<summary style=\"font-size:");
            persistent.append(sEntryFontSize);
            persistent.append("px; font-weight: 500\"> INFOS </summary>\n");
            persistent.append("<ul>\n");
            persistent.append(Objects.requireNonNull(mInfos.get(Target.DASHBOARD)));
            persistent.append("</ul>\n");
            persistent.append("</details>\n");

            persistent.append("<details open>\n");
            persistent.append("<summary style=\"font-size:");
            persistent.append(sEntryFontSize);
            persistent.append("px; font-weight: 500\"> DEBUG </summary>\n");
            persistent.append("<ul>\n");
            persistent.append(Objects.requireNonNull(mDebugs.get(Target.DASHBOARD)));
            persistent.append("</ul>\n");
            persistent.append("</details>\n");

            persistent.append("<details open>\n");
            persistent.append("<summary style=\"font-size:");
            persistent.append(sEntryFontSize);
            persistent.append("px; font-weight: 500\"> TRACES </summary>\n");
            persistent.append("<ul>\n");
            persistent.append(Objects.requireNonNull(mTraces.get(Target.DASHBOARD)));
            persistent.append("</ul>\n");
            persistent.append("</details>\n");

            mDashboard.getTelemetry().addLine(persistent.toString());
        }
    }

    public void raw(Target target, String raw) {
        if (target == Target.DRIVER_STATION && mDriverStation != null) {
            Objects.requireNonNull(mInfos.get(target)).append(raw);
        } else if (target == Target.DASHBOARD && mDashboard != null) {
            Objects.requireNonNull(mInfos.get(target)).append(raw);
        } else if (target == Target.FILE && mFile != null) {
            mFile.info(raw);
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
        mInfos.put(target,new StringBuilder());
        mDebugs.put(target,new StringBuilder());
        mTraces.put(target,new StringBuilder());
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
     * initialize logger from filename
     *
     * @param filename The logging filename (without path and extension)
     * @return the initialized logger
     */
    private Logger initializeFileLogger(String filename) throws IOException {
        Logger result = null;
        String filepath = Environment.getExternalStorageDirectory().getPath()
                + "/FIRST/"
                + filename
                + ".log";

        if(!filename.isEmpty()) {

            result = Logger.getLogger("log-manager");
            FileHandler fileHandler = new FileHandler(filepath,100000,2, true); // Append mode
            SimpleFormatter formatter = new SimpleFormatter() {
                private final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");

                @Override
                public synchronized String format(LogRecord record) {
                    return String.format("[%s] [%s] %s - %s%n",
                            dateFormat.format(new Date(record.getMillis())),  // Timestamp with milliseconds
                            record.getLevel(),  // Log level (INFO, WARNING, SEVERE, etc.)
                            record.getLoggerName(),  // Logger name
                            record.getMessage());  // Log message
                }
            };
            fileHandler.setFormatter(formatter);
            result.setLevel(Level.ALL);
            result.addHandler(fileHandler);
            result.setUseParentHandlers(false);
        }

        return result;
    }


    /**
     * Add an error to a log sink
     *
     * @param target the log sink
     * @param message the error description
     * @param source the stack level from which the error was issued
     */
    private void error(Target target, String message, int source) {

        Integer errorPriority = sLevelToPriority.get(Severity.ERROR);
        Integer filterPriority = sLevelToPriority.get(mLevel);
        if( filterPriority != null && errorPriority != null && filterPriority >= errorPriority) {

            StackTraceElement element = Thread.currentThread().getStackTrace()[source]; // Get caller
            switch (target) {
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
                case FILE:
                    if (mFile != null) {
                        String local = element.getFileName() +
                                ":" +
                                element.getLineNumber() +
                                " - error - " +
                                message;
                        mFile.log(Level.SEVERE, local);
                    }
                    break;
            }
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

        Integer warningPriority = sLevelToPriority.get(Severity.WARNING);
        Integer filterPriority = sLevelToPriority.get(mLevel);
        if( filterPriority != null && warningPriority != null && filterPriority >= warningPriority) {

            StackTraceElement element = Thread.currentThread().getStackTrace()[source]; // Get caller
            switch (target) {
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
                case FILE:
                    if (mFile != null) {
                        String local = element.getFileName() +
                                ":" +
                                element.getLineNumber() +
                                " - " +
                                message;
                        mFile.log(Level.WARNING, local);
                    }
                    break;
            }
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

        Integer metricPriority = sLevelToPriority.get(Severity.METRIC);
        Integer filterPriority = sLevelToPriority.get(mLevel);
        if( filterPriority != null && metricPriority != null && filterPriority >= metricPriority) {

            StackTraceElement element = Thread.currentThread().getStackTrace()[source]; // Get caller
            switch (target) {
                case DASHBOARD:
                    Objects.requireNonNull(mMetrics.get(target)).put(metric, value);
                    if (mDashboard != null) {
                        Telemetry.Item data = mDashboard.getTelemetry().addData(metric, value);
                        //mDashboard.getTelemetry().removeItem(data);
                    }
                    break;
                case DRIVER_STATION:
                    Objects.requireNonNull(mMetrics.get(target)).put(metric, value);
                    break;
                case FILE:
                    if (mFile != null) {
                        String local = element.getFileName() +
                                ":" +
                                element.getLineNumber() +
                                " - metric - " +
                                metric + " : " + value;
                        mFile.log(Level.INFO, local);
                    }
                    break;
            }
        }
    }

    /**
     * Add an info to a log sink
     *
     * @param target the log sink
     * @param message the info message
     * @param source the stack level from which the metric was issued
     */
    private void info(Target target, String message, int source) {

        Integer infoPriority = sLevelToPriority.get(Severity.INFO);
        Integer filterPriority = sLevelToPriority.get(mLevel);
        if( filterPriority != null && infoPriority != null && filterPriority >= infoPriority) {

            StackTraceElement element = Thread.currentThread().getStackTrace()[source]; // Get caller
            switch (target) {
                case DASHBOARD:
                    if (mDashboard != null) {
                        Objects.requireNonNull(mInfos.get(target))
                                .append("<p style=\"color: black; margin-left:30px; list-style-type: square; font-size: ")
                                .append(sInfoFontSize)
                                .append("px\">")
                                .append(element.getFileName())
                                .append(":")
                                .append(element.getLineNumber())
                                .append(" - ")
                                .append(message)
                                .append("</p>");
                    }
                    break;
                case DRIVER_STATION:
                    if (mDriverStation != null) {
                        Objects.requireNonNull(mInfos.get(target))
                                .append(element.getFileName())
                                .append(":")
                                .append(element.getLineNumber())
                                .append(" - info - ")
                                .append(message)
                                .append("\n");
                    }
                    break;
                case FILE:
                    if (mFile != null) {
                        String local = element.getFileName() +
                                ":" +
                                element.getLineNumber() +
                                " - info - " +
                                message;
                        mFile.log(Level.INFO, local);
                    }
                    break;
            }
        }
    }

    /**
     * Add a debug to a log sink
     *
     * @param target the log sink
     * @param message the info message
     * @param source the stack level from which the metric was issued
     */
    private void debug(Target target, String message, int source) {

        Integer debugPriority = sLevelToPriority.get(Severity.DEBUG);
        Integer filterPriority = sLevelToPriority.get(mLevel);
        if( filterPriority != null && debugPriority != null && filterPriority >= debugPriority) {

            StackTraceElement element = Thread.currentThread().getStackTrace()[source]; // Get caller
            switch (target) {
                case DASHBOARD:
                    if (mDashboard != null) {
                        Objects.requireNonNull(mDebugs.get(target))
                                .append("<p style=\"color: black; margin-left:30px; list-style-type: square; font-size: ")
                                .append(sInfoFontSize)
                                .append("px\">")
                                .append(element.getFileName())
                                .append(":")
                                .append(element.getLineNumber())
                                .append(" - ")
                                .append(message)
                                .append("</p>");
                    }
                    break;
                case DRIVER_STATION:
                    if (mDriverStation != null) {
                        Objects.requireNonNull(mDebugs.get(target))
                                .append(element.getFileName())
                                .append(":")
                                .append(element.getLineNumber())
                                .append(" - debug - ")
                                .append(message)
                                .append("\n");
                    }
                    break;
                case FILE:
                    if (mFile != null) {
                        String local = element.getFileName() +
                                ":" +
                                element.getLineNumber() +
                                " - debug - " +
                                message;
                        mFile.log(Level.INFO, local);
                    }
                    break;
            }
        }
    }

    /**
     * Add a trace to a log sink
     *
     * @param target the log sink
     * @param message the info message
     * @param source the stack level from which the metric was issued
     */
    private void trace(Target target, String message, int source) {

        Integer tracePriority = sLevelToPriority.get(Severity.TRACE);
        Integer filterPriority = sLevelToPriority.get(mLevel);
        if( filterPriority != null && tracePriority != null && filterPriority >= tracePriority) {

            StackTraceElement element = Thread.currentThread().getStackTrace()[source]; // Get caller
            switch (target) {
                case DASHBOARD:
                    if (mDashboard != null) {
                        Objects.requireNonNull(mTraces.get(target))
                                .append("<p style=\"color: black; margin-left:30px; list-style-type: square; font-size: ")
                                .append(sInfoFontSize)
                                .append("px\">")
                                .append(element.getFileName())
                                .append(":")
                                .append(element.getLineNumber())
                                .append(" - ")
                                .append(message)
                                .append("</p>");
                    }
                    break;
                case DRIVER_STATION:
                    if (mDriverStation != null) {
                        Objects.requireNonNull(mTraces.get(target))
                                .append(element.getFileName())
                                .append(":")
                                .append(element.getLineNumber())
                                .append(" - trace - ")
                                .append(message)
                                .append("\n");
                    }
                    break;
                case FILE:
                    if (mFile != null) {
                        String local = element.getFileName() +
                                ":" +
                                element.getLineNumber() +
                                " - trace - " +
                                message;
                        mFile.log(Level.INFO, local);
                    }
                    break;
            }
        }
    }
}
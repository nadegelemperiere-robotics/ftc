/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Configuration management
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.configuration;

/* System includes */
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.ArrayList;
import java.util.Map;

/* Android includes */
import android.os.Environment;

/* Json includes */
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONArray;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class Configuration {

    // Default working configuration
    static protected String         sDefaultConfiguration = "";
    static protected Configuration  sInstance = null;

    // Status
    boolean                         mIsValid;

    // Registered configurables to read and write configuration
    final Map<String, Configurable> mConfigRegistry;

    // Loggers
    LogManager                      mLogger;
    String                          mFilename;

    /**
     * Configuration singleton access
     *
     */
    public  static  Configuration getInstance() {
        if(sInstance == null) { sInstance = new Configuration(); }
        return sInstance;
    }

    /**
     * Configuration constructor
     *
     */
    public Configuration() {

        mConfigRegistry = new LinkedHashMap<>();
        mIsValid        = false;

    }

    /**
     * Configuration constructor
     *
     * @param logger logger
     */
    public Configuration(LogManager logger) {

        mLogger         = logger;
        mConfigRegistry = new LinkedHashMap<>();
        mIsValid        = false;

    }

    /**
     * Configuration logger association
     *
     * @param logger logger
     */
    public void logger(LogManager logger) {
        mLogger         = logger;
    }

    /**
     * Registers an object to be configured
     *
     * @param key configuration key corresponding to the object configuration
     * @param configurable object to configure
     */
    public void register(String key, Configurable configurable) {
        mConfigRegistry.put(key, configurable);
    }

    /**
     * Checks if configuration is valid
     *
     * @return true if configuration is valid, false otherwise
     */
    public boolean isValid() { return mIsValid; }


    /**
     * Reads configuration from the default configuration file
     *
     */
    public void read() {

        String filename = Environment.getExternalStorageDirectory().getPath()
                + "/FIRST/"
                + Configuration.sDefaultConfiguration
                + ".json";
        this.read(filename);
    }

    /**
     * Reads configuration from a given configuration file
     *
     * @param filename : filename containing configuration
     */
    public void read(String filename) {

        mFilename = Configuration.getRawFilename(filename);
        mIsValid  = true;

        StringBuilder content = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(new FileInputStream(filename), StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) {
                content.append(line);
            }
        }
        catch(IOException e) {
            mLogger.error("Configuration file " + filename + " reading failed");
            mIsValid = false;
        }

        // Parse JSON content
        JSONObject configuration = null;
        try {
            configuration = new JSONObject(content.toString());
        }
        catch(JSONException e) {
            mLogger.error("Configuration file " + filename + " is not json formatted");
            mIsValid = false;
        }

        if(mIsValid) {

            // Loop into configurable to load their configuration
            for (Map.Entry<String, Configurable> configurable : mConfigRegistry.entrySet()) {

                try {

                    // Split key between keys and indexes
                    String[] keys = configurable.getKey().split("[.\\[\\]]");
                    Object data = configuration;
                    for (String key : keys) {

                        if (!key.isEmpty()) {

                            if (data instanceof JSONObject) {
                                data = ((JSONObject) data).get(key);
                            } else if (data instanceof JSONArray) {
                                int index = Integer.parseInt(key);
                                data = ((JSONArray) data).get(index);
                            }
                        }

                    }

                    if (data instanceof JSONObject) {
                        configurable.getValue().read((JSONObject) data);
                    } else {
                        mLogger.error("Target key " + configurable.getKey() + " does not lead to an object" );
                        mIsValid = false;
                    }
                }
                catch (JSONException | NumberFormatException e) {
                    mLogger.error("Configuration " + configurable.getKey() + " can not be read" );
                    mIsValid = false;
                }
            }

        }
        for (Map.Entry<String, Configurable> configurable : mConfigRegistry.entrySet()) {
            if(!configurable.getValue().isConfigured()) {
                mLogger.error("Configuration " + configurable.getKey() + " is not valid" );
                mIsValid = false;
            }
        }
    }

    /**
     * Reads configuration from a given configuration file
     *
     * @param filename : filename containing configuration
     */
    public void write(String filename) {

        JSONObject configuration = new JSONObject();

        // Loop into configurable to write their configuration
        for (Map.Entry<String, Configurable> configurable : mConfigRegistry.entrySet()) {

            if(configurable.getValue().isConfigured()) {

                try {
                    // Split key between topics - Indexes still belong to the key they come from,
                    String[] keys = configurable.getKey().split("[.]");
                    Object data = configuration;
                    for (String key : keys) {

                        if (!key.isEmpty()) {

                            if (data == null) { throw new RuntimeException(); }

                            JSONObject json = ((JSONObject) data);

                            String k;
                            List<Integer> indexes = new ArrayList<>();
                            k = Configuration.getKeyAndIndexes(key, indexes, mLogger);

                            if (!json.has(k) && indexes.isEmpty())  { json.put(k, new JSONObject()); }
                            if (!json.has(k) && !indexes.isEmpty()) { json.put(k, new JSONArray());  }

                            Object temp = json.get(k);
                            if ((temp instanceof JSONObject) && (!indexes.isEmpty())) {
                                throw new RuntimeException();
                            }
                            if ((temp instanceof JSONArray) && (indexes.isEmpty())) {
                                throw new RuntimeException();
                            }

                            if (indexes.isEmpty()) {
                                if (!(temp instanceof JSONObject)) {  throw new RuntimeException(); }
                                data = temp;
                            } else {
                                if (!(temp instanceof JSONArray)) {  throw new RuntimeException(); }
                                JSONArray array = ((JSONArray) temp);

                                for (int i_index = 0; i_index < (indexes.size() - 1); i_index++) {
                                    if (array.length() <= indexes.get(i_index)) {
                                        for (int j_index = array.length(); j_index <= indexes.get(i_index); j_index++) {
                                            array.put(new JSONArray());
                                        }
                                    }

                                    temp = array.get(indexes.get(i_index));
                                    array = ((JSONArray) temp);
                                }
                                if (array.length() <= indexes.get(indexes.size() - 1)) {
                                    for (int j_index = array.length(); j_index <= indexes.get(indexes.size() - 1); j_index++) {
                                        array.put(new JSONObject());
                                    }

                                    temp = array.get(indexes.get(indexes.size() - 1));
                                    data = temp;
                                }
                            }
                        }
                    }
                    configurable.getValue().write((JSONObject) data);
                } catch (RuntimeException | JSONException e) {
                    mLogger.error("Configuration formatting failed");
                }
            }
        }

        // Write JSON to file
        try {
            BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filename), StandardCharsets.UTF_8));
            writer.write(configuration.toString(4)); // 4 is the indentation level for pretty printing
            writer.close();
        }
        catch(JSONException | IOException e) { mLogger.error("Unable to write json file"); }
    }


    public void log() {

        // Log to driver station
        if(mIsValid) {
            mLogger.raw(LogManager.Target.DRIVER_STATION, "CNF " + mFilename + " is valid");
        }
        else {
            mLogger.raw(LogManager.Target.DRIVER_STATION, "CNF " + mFilename + " is not valid");
        }

        // Log to dashboard
        StringBuilder confstring = new StringBuilder();
        confstring.append("-------------------------\n");
        if(mIsValid) {
            confstring.append("<p style=\"color: green; font-size: 14px\"> Conf ")
                    .append(mFilename)
                    .append(" is valid</p>");
        }
        else {
            confstring.append("<p style=\"color: red; font-size: 14px\"> Conf ")
                    .append(mFilename)
                    .append(" is not valid</p>");
        }

        // Loop into configurable to write their configuration
        for (Map.Entry<String, Configurable> configurable : mConfigRegistry.entrySet()) {

            confstring.append("-------------------------\n")
                      .append("<details>\n")
                      .append("<summary style=\"font-size: 12px; font-weight: 500\"> ")
                      .append(configurable.getKey().toUpperCase())
                      .append(" </summary>\n")
                      .append("<ul>\n")
                      .append(configurable.getValue().logConfigurationHTML())
                      .append("</ul>\n")
                      .append("</details>\n");

        }

        mLogger.raw(LogManager.Target.DASHBOARD,confstring.toString());

        // Log to file
        confstring = new StringBuilder();
        confstring.append("\n-------------------------\n");
        if(mIsValid) {
            confstring.append(mFilename)
                    .append(" is valid\n");
        }
        else {
            confstring.append(mFilename)
                    .append(" is not valid\n");
        }
        // Loop into configurable to write their configuration
        for (Map.Entry<String, Configurable> configurable : mConfigRegistry.entrySet()) {

            confstring.append("-------------------------\n")
                    .append(configurable.getKey().toUpperCase())
                    .append("\n")
                    .append(configurable.getValue().logConfigurationText("--"))
                    .append("\n");

        }

        mLogger.raw(LogManager.Target.FILE,confstring.toString());
    }

    private static String getRawFilename(String filename) {
        String result;

        File file = new File(filename);
        result = file.getName();

        int dotIndex = result.lastIndexOf(".");
        result = result.substring(0,dotIndex);

        return result;
    }

    private static String getKeyAndIndexes(String key, List<Integer> indexes, LogManager logger) {

        String result = key;
        indexes.clear();

        if (key.contains("[") && key.contains("]")) {
            result = key.substring(0, key.indexOf("["));
            String[] inds = key.substring(key.indexOf("["), key.length() - 1).split("[\\[\\]]");

            for (String index : inds) {
                if(!index.isEmpty()) {
                    try {
                        Integer i = Integer.parseInt(index);
                        indexes.add(i);
                    }
                    catch(NumberFormatException e) {
                        logger.error(key + " does contains an invalid index");
                    }
                }
            }
        }

        return result;

    }


}

/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Class mapping controller inputs to robot behavior
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.dispatcher;

/* System includes */
import java.util.Map;
import java.util.List;
import java.util.ArrayList;

/* Json includes */
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONArray;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;
import org.firstinspires.ftc.core.tools.Condition;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

/* Components includes */
import org.firstinspires.ftc.core.components.controllers.Controller;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Robot;

public class Dispatcher implements Configurable {

    static  final   String              sControllersKey = "controllers";
    static  final   String              sCommandsKey    = "commands";

    final   protected LogManager            mLogger;

    boolean                                 mConfigurationValid;

    protected final Robot                   mRobot;

    protected final Map<String,Controller>  mControllers;

    final List<Command>                     mCommands;

    /**
     * ControlMapper constructor
     *
     * @param controllers dynamic controllers for commands
     * @param robot robot to command
     * @param logger logger
     */
    public Dispatcher(Map<String,Controller> controllers, Robot robot, LogManager logger) {
        mLogger      = logger;
        mControllers = controllers;
        mRobot       = robot;
        mConfigurationValid = true;
        mCommands = new ArrayList<>();
        this.commands();
    }

    /**
     * Reads control configuration
     *
     * @param reader : JSON object containing configuration
     */
    public void read(JSONObject reader) {

        mConfigurationValid = true;

        if(reader.has(sControllersKey)) {
            try {
                JSONObject controllers = reader.getJSONObject(sControllersKey);
                for (Map.Entry<String, Controller> controller : mControllers.entrySet()) {
                    if(controllers.has(controller.getKey())) {
                        JSONObject object = controllers.getJSONObject(controller.getKey());
                        controller.getValue().read(object);
                        if(!controller.getValue().isConfigured()) { mConfigurationValid = false; }
                    }
                }
            } catch(JSONException e) {
                mLogger.error("Error in configuration reading");
                mConfigurationValid = false;
            }
        }

        if(reader.has(sCommandsKey)) {
            try {
                JSONArray commands = reader.getJSONArray(sCommandsKey);
                for (int i_command = 0; i_command < commands.length(); i_command ++) {
                    Command command = new Command(mControllers, mRobot, mLogger);
                    command.read((JSONObject) commands.get(i_command));
                    mCommands.add(command);
                }
            } catch(JSONException e) {
                mLogger.error("Error in configuration reading");
                mConfigurationValid = false;
            }
        }

    }

    /**
     * Writes control configuration
     *
     * @param writer : JSON object to store configuration
     */
    public void write(JSONObject writer) {

        JSONObject controllers = new JSONObject();
        try {
            for (Map.Entry<String, Controller> controller : mControllers.entrySet()) {
                JSONObject object = new JSONObject();
                controller.getValue().write(object);
                controllers.put(controller.getKey(), object);
            }
        } catch(JSONException e) {
                mLogger.error("Error in configuration reading");
                mConfigurationValid = false;
        }

        try {
            writer.put(sControllersKey, controllers);
        } catch(JSONException e) {
            mLogger.error("Error in configuration reading");
            mConfigurationValid = false;
        }

    }

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

        StringBuilder result = new StringBuilder();

        result.append("<details>\n")
                .append("<summary style=\"font-size: 12px; font-weight: 500\"> CONTROLLERS </summary>\n")
                .append("<ul>\n");

        for (Map.Entry<String, Controller> controller : mControllers.entrySet()) {

            result.append("<li>\n")
                    .append("<summary style=\"font-size: 12px; font-weight: 500\">")
                    .append(controller.getKey().toUpperCase())
                    .append("</summary>\n")
                    .append("<ul>\n")
                    .append(controller.getValue().logConfigurationHTML())
                    .append("</ul>\n")
                    .append("</details>\n")
                    .append("</li>\n");
        }

        result.append("</ul>\n")
                .append("</details>\n");

        return result.toString();
    }

    /**
     * Configuration logging into text
     *
     * @return configuration as string
     */
    public String  logConfigurationText(String header) {

        StringBuilder result = new StringBuilder(header +
                "> controllers : \n");

        for (Map.Entry<String, Controller> controller : mControllers.entrySet()) {

            result.append(header)
                    .append("--> ")
                    .append(controller.getKey())
                    .append("\n")
                    .append(controller.getValue().logConfigurationText(header + "--"));
        }

        return result.toString();
    }

    /**
     * Function to loop into commands
     */
    public void update() {

        mLogger.info(LogManager.Target.FILE,"start");

        for(int i_command = 0; i_command < mCommands.size(); i_command ++) {
            mCommands.get(i_command).execute();
        }

        mLogger.info(LogManager.Target.FILE,"stop");

    }

    /**
     * Add command through code
     *
     * @param condition command condition
     * @param action command action
     */
    protected void registerCommand(Condition condition, Runnable action) {
        mCommands.add(new Command(condition, action, mLogger));
    }

    /**
     * Function to overload to add command in scheduler
     *
     */
    protected void commands() {

    }
}

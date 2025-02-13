/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Class mapping controller inputs to robot behavior
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.controller;

/* System includes */
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.security.InvalidParameterException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/* Json includes */
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

/* Components includes */
import org.firstinspires.ftc.core.components.controllers.Controller;
import org.firstinspires.ftc.core.components.controllers.Axis;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Robot;

public class Command implements Configurable {

    static  final String        sActionKey       = "action";
    static  final String        sConditionKey    = "condition";
    static  final String        sButtonKey       = "button";
    static  final String        sStateKey        = "state";
    static  final String        sControllerKey   = "controller";
    static  final String        sTriggerKey      = "trigger";
    static  final String        sOperationKey    = "operation";
    static  final String        sThresholdKey    = "threshold";

    LogManager                  mLogger;

    boolean                     mConfigurationValid;

    Map<String,Controller>      mControllers;

    Robot                       mRobot;

    Condition                   mCondition;

    Runnable                    mAction;


    /**
     * ControlMapper constructor
     *
     * @param controllers controllers to analyze
     * @param robot robot to analyze
     * @param logger logger
     */
    public Command(Map<String,Controller> controllers, Robot robot, LogManager logger) {
        mLogger         = logger;
        mControllers    = controllers;
        mRobot          = robot;
        mCondition      = new Condition(() -> true);

        mConfigurationValid = false;
    }

    /**
     * ControlMapper constructor
     *
     * @param condition condition to launch action
     * @param action action to launch
     * @param logger logger
     */
    public Command(Condition condition, Runnable action, LogManager logger) {
        mLogger         = logger;
        mCondition      = condition;
        mControllers    = new LinkedHashMap<>();
        mRobot          = null;
        mAction         = action;

        mConfigurationValid = true;
    }

    public void register(String topic) {}

    /**
     * Reads control configuration
     *
     * @param reader : JSON object containing configuration
     */
    public void read(JSONObject reader) {

        mConfigurationValid = true;

        if(reader.has(sConditionKey)) {
            try {
                JSONObject object = reader.getJSONObject(sConditionKey);
                mCondition = this.readCondition(object);
            } catch(JSONException e) {
                mLogger.error("Error in configuration reading");
                mConfigurationValid = false;
            }
        }

        if(reader.has(sActionKey)) {
            try {
                String action = reader.getString(sActionKey);
                Method method = mRobot.getClass().getMethod(action);
                mAction = () -> {
                    try {
                        method.invoke(mRobot);
                    }
                    catch(IllegalAccessException | IllegalArgumentException | InvocationTargetException ignored) { }
                };

            } catch(JSONException | NoSuchMethodException e) {
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

        return "";
    }

    /**
     * Configuration logging into text
     *
     * @return configuration as string
     */
    public String  logConfigurationText(String header) {

        return "";
    }

    /**
     * Execute command
     */
    public void execute() {

        mLogger.info(LogManager.Target.FILE,"start");

        if(mCondition.evaluate()) { mAction.run(); }

        mLogger.info(LogManager.Target.FILE,"stop");

    }



    private Condition readCondition(JSONObject object)
    {
        Condition result = null;

        try {

            boolean isVariable = true;

            if(object.has("and")) {
                JSONArray array = object.getJSONArray("and");
                List<Condition> conditions = new ArrayList<>();
                for (int i_index = 0; i_index < array.length(); i_index++) {
                    conditions.add(this.readCondition(array.getJSONObject(i_index)));
                }
                result = Condition.and(conditions);
            }
            else if(object.has("nand")) {
                JSONArray array = object.getJSONArray("nand");
                List<Condition> conditions = new ArrayList<>();
                for (int i_index = 0; i_index < array.length(); i_index++) {
                    conditions.add(this.readCondition(array.getJSONObject(i_index)));
                }
                result = Condition.nand(conditions);
            }
            else if(object.has("or")) {
                JSONArray array = object.getJSONArray("or");
                List<Condition> conditions = new ArrayList<>();
                for (int i_index = 0; i_index < array.length(); i_index++) {
                    conditions.add(this.readCondition(array.getJSONObject(i_index)));
                }
                result = Condition.or(conditions);
            }
            else if(object.has("nor")) {
                JSONArray array = object.getJSONArray("nor");
                List<Condition> conditions = new ArrayList<>();
                for (int i_index = 0; i_index < array.length(); i_index++) {
                    conditions.add(this.readCondition(array.getJSONObject(i_index)));
                }
                result = Condition.nor(conditions);
            }
            else if(object.has("xor")) {
                JSONArray array = object.getJSONArray("xor");
                List<Condition> conditions = new ArrayList<>();
                for (int i_index = 0; i_index < array.length(); i_index++) {
                    conditions.add(this.readCondition(array.getJSONObject(i_index)));
                }
                result = Condition.xor(conditions);
            }
            else if(object.has("non")) {
                JSONObject object2 = object.getJSONObject("not");
                Condition condition = this.readCondition(object2);
                result = Condition.not(condition);
            }
            else {

                if(!object.has(sControllerKey)) throw new InvalidParameterException("Missing controller for condition leaf");
                Controller controller = mControllers.get(object.getString(sControllerKey));
                if(controller == null) throw new InvalidParameterException("Controller not found for condition leaf");

                if(object.has(sButtonKey)) {
                    if(!object.has(sStateKey)) throw new InvalidParameterException("Missing state for condition leaf button");
                    Field field = controller.buttons.getClass().getField(object.getString(sButtonKey));
                    Method method = field.getType().getMethod(object.getString(sStateKey));
                    Object button = field.get(controller.buttons);
                    if(button == null) throw new InvalidParameterException("Button " + object.getString(sButtonKey) + " not found in controller buttons");

                    result = new Condition(() -> {
                        boolean res = false;
                        try {
                            Object temp = method.invoke(button);
                            if(temp != null)  { res = (boolean)temp; }
                        }
                        catch(IllegalAccessException | InvocationTargetException ignored) {
                        }
                        return res;
                    });
                }

                else if (object.has(sTriggerKey)) {
                    if(!object.has(sOperationKey)) throw new InvalidParameterException("Missing operation for condition leaf trigger");
                    if(!object.has(sThresholdKey)) throw new InvalidParameterException("Missing threshold for condition leaf trigger");

                    double threshold = object.getDouble(sThresholdKey);
                    String operation = object.getString(sOperationKey);

                    Field field = controller.axes.getClass().getField(object.getString(sTriggerKey));
                    Object axis = field.get(controller.axes);
                    if(axis == null) throw new InvalidParameterException("Trigger " + object.getString(sTriggerKey) + " not found in controller axes");

                    if(operation.equals("lesserThan")) {
                        result = new Condition(() -> {
                            return (((Axis) axis).value() < threshold);
                        });
                    }
                    else if(operation.equals("greaterThan")) {
                        result = new Condition(() -> {
                            return (((Axis) axis).value() > threshold);
                        });
                    }
                    else throw new InvalidParameterException("Unmanged operation " + operation);
                }
            }

        }
        catch(JSONException | IllegalAccessException | NoSuchFieldException | NoSuchMethodException e) {
            mLogger.error(e.getMessage());
        }


        return result;
    }

}

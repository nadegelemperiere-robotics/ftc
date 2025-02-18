/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Class managing a command
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.dispatcher;

/* System includes */
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Iterator;
import java.util.Map;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.security.InvalidParameterException;

/* Json includes */
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;
import org.firstinspires.ftc.core.tools.Condition;

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

    final LogManager                mLogger;

    boolean                         mConfigurationValid;

    final Map<String,Controller>    mControllers;

    final Robot                     mRobot;

    Condition                       mCondition;
    Runnable                        mAction;


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

    /**
     * Reads control configuration
     *
     * @param reader : JSON object containing configuration
     */
    public void read(JSONObject reader) {

        mConfigurationValid = true;
        int nbParams = reader.length();

        if(reader.has(sConditionKey)) {
            try {
                JSONObject object = reader.getJSONObject(sConditionKey);
                mCondition = this.readCondition(object);
            } catch(JSONException e) {
                mLogger.error("Error in configuration reading");
                mConfigurationValid = false;
            }
            nbParams --;
        }

        if(reader.has(sActionKey)) {
            try {
                nbParams --;
                String action = reader.getString(sActionKey);
                if(nbParams == 0) {
                    Method method = mRobot.getClass().getMethod(action);
                    mAction = () -> {
                        try {
                            method.invoke(mRobot);
                        }
                        catch(IllegalAccessException | IllegalArgumentException | InvocationTargetException ignored) { }
                    };
                }
                else {
                    Method[] methods = mRobot.getClass().getMethods();
                    Method temp = null;
                    for (Method value : methods) {
                        if (value.getName().equals(action)) {
                            temp = value;
                        }
                    }
                    Method method = temp;
                    ParamEvaluator evaluator = new ParamEvaluator(reader, mControllers, mLogger);

                    mAction = () -> {
                        try {
                            Object[] parameters = evaluator.evaluate();
                            method.invoke(mRobot, parameters);
                        } catch (IllegalAccessException | IllegalArgumentException |
                                 InvocationTargetException | NullPointerException ignored) {
                        }
                    };
                }

            } catch(JSONException | NoSuchMethodException e) {
                mLogger.error(e.getMessage());
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

        mLogger.error("Unable to write commands");

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

        if(mCondition.evaluate()) { mAction.run(); }

    }

    /**
     * Reads JSON object and create condition recursively
     *
     * @param object : JSON object to read condition from
     */
    private Condition readCondition(JSONObject object)
    {
        Condition result = null;

        try {

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
                        result = new Condition(() -> (((Axis) axis).value() < threshold));
                    }
                    else if(operation.equals("greaterThan")) {
                        result = new Condition(() -> (((Axis) axis).value() > threshold));
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

class ParamEvaluator {

    static  final String        sActionKey       = "action";
    static  final String        sConditionKey    = "condition";
    static  final String        sControllerKey   = "controller";
    static  final String        sInputKey        = "input";
    static  final String        sScaleKey        = "scale";

    final LogManager                mLogger;

    final List<Object>              mParameters;
    final List<Runnable>            mProcessors;
    final Map<String,Controller>    mControllers;

    /**
     * Parameter evaluator constructor
     *
     * @param object : JSON object to read parameters from
     * @param controllers : controller objects to read dynamically
     * @param logger : logger
     */
    public ParamEvaluator(JSONObject object, Map<String,Controller> controllers, LogManager logger) {
        mParameters     = new ArrayList<>();
        mProcessors     = new ArrayList<>();
        mControllers    = controllers;
        mLogger         = logger;
        this.read(object);
    }

    /**
     * Reads JSON object and create parameters from it
     *
     * @param object : JSON object to read parameters from
     */
    public void read(JSONObject object) {

        Iterator<String> keys = object.keys();
        List<String> sortedKeys = new ArrayList<>();
        while (keys.hasNext()) {
            String key = keys.next();
            sortedKeys.add(key);
        }
        Collections.sort(sortedKeys);

        keys = sortedKeys.iterator();
        while (keys.hasNext()) {
            String key = keys.next();
            if (!key.equals(sActionKey) && !key.equals(sConditionKey)) {
                boolean found = false;
                try {
                    double parameter = object.getDouble(key);

                    mProcessors.add(() -> mParameters.add(parameter));
                    found = true;
                } catch (JSONException ignored) {
                }
                if(!found) {
                    try {
                        JSONObject parameter = object.getJSONObject(key);

                        if (!parameter.has(sControllerKey))
                            throw new InvalidParameterException("Missing controller for condition leaf");
                        Controller controller = mControllers.get(parameter.getString(sControllerKey));
                        if (controller == null)
                            throw new InvalidParameterException("Controller not found for condition leaf");

                        if (!parameter.has(sInputKey))
                            throw new InvalidParameterException("Missing controller for condition leaf");

                        double scale = parameter.has(sScaleKey) ? parameter.getDouble(sScaleKey) : 1.0;

                        Field field = controller.axes.getClass().getField(parameter.getString(sInputKey));
                        Object axis = field.get(controller.axes);
                        if (axis == null)
                            throw new InvalidParameterException("Trigger " + parameter.getString(sInputKey) + " not found in controller axes");
                        Axis trigger = ((Axis) axis);

                        mProcessors.add(() -> mParameters.add(scale * trigger.value()));

                    } catch (JSONException | NoSuchFieldException | IllegalAccessException e) {
                        mLogger.error(e.getMessage());
                    }
                }

            }
        }
    }

    /**
     * Evaluate parameters current values
     */
    public Object[]    evaluate() {

        mParameters.clear();
        for (int i_processor = 0; i_processor < mProcessors.size(); i_processor++) {
            mProcessors.get(i_processor).run();
        }
        return mParameters.toArray();
    }

}

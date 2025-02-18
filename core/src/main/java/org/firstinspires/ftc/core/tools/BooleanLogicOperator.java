/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Boolean logic evaluation tool
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.tools;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.List;
import java.util.ArrayList;
import java.util.Objects;
import java.util.Iterator;

/* Json includes */
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONArray;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

public class BooleanLogicOperator implements Configurable {

    private static final Map<BooleanNode.Operation , String > sOperatorToConf = Map.of(
            BooleanNode.Operation.AND,  "and",
            BooleanNode.Operation.OR,   "or",
            BooleanNode.Operation.NOT,  "not",
            BooleanNode.Operation.XOR,  "xor",
            BooleanNode.Operation.NAND, "nand",
            BooleanNode.Operation.NOR,  "nor"
            );

    // Status
    boolean                         mConfigurationValid;

    final   LogManager              mLogger;

    final   Map<Integer,JSONObject> mValues;
    BooleanNode                     mTree;
    Integer                         mCurrentIndex;



    /**
     * Builds a boolean logic operator
     *
     * @param logger logger
     */
    public BooleanLogicOperator(LogManager logger) {

        mConfigurationValid = true;

        mLogger             = logger;

        mValues             = new LinkedHashMap<>();
        mTree               = null;
        mCurrentIndex       = -1;

    }

    /**
     * Returns values needed by the logic operator to process
     *
     * @return map between value index and the JSON object detailed information on variable
     */
    public Map<Integer, JSONObject> values() { return mValues; }

    /**
     * Evaluate boolean logic for a set of values
     *
     * @param values : Values for operator variables
     */
    public Boolean  evaluate(Map<Integer,Boolean> values) {
        return this.evaluateBooleanLogic(mTree, values);
    }

    /**
     * Configuration checking
     *
     * @return true if object is correctly configured, false otherwise
     */
    public boolean  isConfigured() {
        return mConfigurationValid;
    }

    /**
     * Configuration logging into HTML
     *
     * @return configuration as html string
     */
    public String   logConfigurationHTML() {

        JSONObject object = new JSONObject();
        this.writeBooleanLogic(mTree,object);
        return object.toString();
    }
    /**
     * Configuration logging into text
     *
     * @return configuration as basic string
     */
    public String   logConfigurationText(String header) {

        JSONObject object = new JSONObject();
        this.writeBooleanLogic(mTree,object);
        return header + object;
    }

    /**
     * Reads boolean logic configuration
     *
     * @param reader : JSON object containing configuration
     */
    public void     read(JSONObject reader) {

        mConfigurationValid = true;
        mCurrentIndex = 0;
        mTree = this.parseBooleanLogic(reader);
    }

    /**
     * Writes log manager configuration
     *
     * @param writer : JSON object to store configuration
     */
    public void     write(JSONObject writer) {
        this.writeBooleanLogic(mTree,writer);
    }

    /**
     * Logic evaluation recursive function
     *
     * @param node : node to evaluate
     * @param values : leaf values to use for evaluation
     */
    private Boolean evaluateBooleanLogic(BooleanNode node, Map<Integer, Boolean> values) {

        Boolean result;
        if (node.isVariable()) result = values.getOrDefault(node.index(), false);
        else {

            List<Boolean> compute = new ArrayList<>();
            List<BooleanNode> children = node.children();
            for (int i_node = 0; i_node < children.size(); i_node++) {
                compute.add(this.evaluateBooleanLogic(children.get(i_node), values));
            }

            switch (node.operation()) {
                case AND:
                    result = true;
                    for (int i_node = 0; i_node < compute.size(); i_node++) {
                        result = result && compute.get(i_node);
                    }
                    break;
                case NAND:
                    result = true;
                    for (int i_node = 0; i_node < compute.size(); i_node++) {
                        result = result && compute.get(i_node);
                    }
                    result = !result;
                    break;
                case OR:
                    result = false;
                    for (int i_node = 0; i_node < compute.size(); i_node++) {
                        result = result || compute.get(i_node);
                    }
                    break;
                case XOR:
                    int numberTrue = 0;
                    for (int i_node = 0; i_node < compute.size(); i_node++) {
                        if (compute.get(i_node)) {
                            numberTrue++;
                        }
                    }
                    result = ((numberTrue % 2) == 1);
                    break;
                case NOR:
                    result = false;
                    for (int i_node = 0; i_node < compute.size(); i_node++) {
                        result = result || compute.get(i_node);
                    }
                    result = !result;
                    break;
                case NOT:
                    result = !(compute.get(0));
                    break;
                default:
                    throw new IllegalArgumentException("Invalid operation: " + node.operation());
            }
        }

        return result;
    }

    /**
     * Logic parsing recursive function
     *
     * @param object : JSON object to parse
     */
    private BooleanNode parseBooleanLogic(JSONObject object) {

        BooleanNode result = null;
        BooleanNode.Operation operator = BooleanNode.Operation.AND;

        try {

            boolean isVariable = true;
            for (BooleanNode.Operation op : BooleanNode.Operation.values()) {
                String key = sOperatorToConf.get(op);
                if (object.has(key)) {
                    isVariable = false;
                    operator = op;
                }
            }
            if (isVariable) {

                Integer index = -1;
                for (Map.Entry<Integer, JSONObject> value : mValues.entrySet()) {
                    if(BooleanLogicOperator.areEqual(object, value.getValue())) { index = value.getKey(); }
                }
                if(index == -1) {
                    index = mCurrentIndex;
                    mValues.put(index,object);
                    mCurrentIndex ++;
                }
                result =  new BooleanNode(index);
            } else {
                List<BooleanNode> nodes = new ArrayList<>();
                if (operator != BooleanNode.Operation.NOT) {
                    JSONArray array = object.getJSONArray(Objects.requireNonNull(sOperatorToConf.get(operator)));
                    for (int i_index = 0; i_index < array.length(); i_index++) {
                        nodes.add(this.parseBooleanLogic(array.getJSONObject(i_index)));
                    }
                }
                else {
                    JSONObject obj = object.getJSONObject(Objects.requireNonNull(sOperatorToConf.get(operator)));
                    nodes.add(this.parseBooleanLogic(obj));
                }
                result = new BooleanNode(operator, nodes);
            }
        }
        catch(JSONException e) {
            mLogger.error("Invalid boolean operator structure");
        }


        return result;
    }

    /**
     * Logic writing recursive function
     *
     * @param node : node to write
     * @param object : JSON object to write
     */
    private void writeBooleanLogic(BooleanNode node, JSONObject object) {

        try {

            if (node.isVariable()) {
                JSONObject obj = mValues.get(node.index());
                assert obj != null;
                Iterator<String> keys = obj.keys();
                while (keys.hasNext()) {
                    String key = keys.next();
                    object.put(key, obj.get(key));
                }
            }
            else {

                switch (node.operation()) {
                    case AND :
                    case OR :
                    case XOR :
                    case NOR :
                    case NAND :

                        JSONArray array = new JSONArray();
                        List<BooleanNode> children = node.children();
                        for (int i_children = 0; i_children < children.size(); i_children++) {
                            JSONObject obj = new JSONObject();
                            this.writeBooleanLogic(children.get(i_children), obj);
                            array.put(obj);
                        }
                        object.put(Objects.requireNonNull(sOperatorToConf.get(node.operation())), array);
                        break;

                    case NOT:

                        JSONObject obj = new JSONObject();
                        List<BooleanNode> child = node.children();
                        this.writeBooleanLogic(child.get(0), obj);
                        object.put(Objects.requireNonNull(sOperatorToConf.get(node.operation())), obj);
                        break;

                }
            }
        }
        catch(NullPointerException | JSONException e) {
            mLogger.error("Invalid boolean operator structure");
        }
    }

    private static Boolean areEqual(JSONObject object1, JSONObject object2) {
        Boolean result = true;

        try {
            Iterator<String> keys = object1.keys();
            while (keys.hasNext()) {
                String key = keys.next();
                if (!object2.has(key)) { result = false; }
                else if (!(object1.getString(key).equals(object2.getString(key)))) { result = false; }
            }

            keys = object2.keys();
            while (keys.hasNext()) {
                String key = keys.next();
                if (!object1.has(key)) { result = false; }
                else if (!(object1.getString(key).equals(object2.getString(key)))) { result = false; }
            }
        }
        catch (JSONException e) { result = false; }

        return result;

    }

}

class BooleanNode {

    public enum Operation  {
        AND,
        OR,
        NOT,
        XOR,
        NAND,
        NOR
    }

    Operation           mOperation;
    final   boolean     mIsVariable;
    Integer             mIndex;
    List<BooleanNode>   mChildren; // Children nodes
    
    BooleanNode(Operation operation, List<BooleanNode> children) {
        mIsVariable = false;
        mOperation = operation;
        mChildren  = children;
    }

    BooleanNode(Integer index) {
        mIsVariable = true;
        mIndex = index;
    }

    boolean             isVariable() { return mIsVariable; }
    Integer             index()      { return mIndex; }
    List<BooleanNode>   children()   { return mChildren; }
    Operation           operation()  { return mOperation; }
}

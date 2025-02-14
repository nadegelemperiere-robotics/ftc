/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Hardware manager
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.robot;

/* System includes */
import java.util.Map;
import java.util.LinkedHashMap;
import java.util.Iterator;
import java.util.List;
import java.util.ArrayList;

/* Json includes */
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONArray;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Components includes */
import org.firstinspires.ftc.core.components.motors.MotorComponent;
import org.firstinspires.ftc.core.components.servos.ServoComponent;


public class Tuning extends Hardware {

    Map<String, MotorComponent> mSingleMotors;
    Map<String, List<String>>   mMotorMapping;

    Map<String, ServoComponent> mSingleServos;
    Map<String, List<String>>   mServoMapping;


    public Tuning(HardwareMap map, LogManager logger) {
        super(map, logger);

        mSingleMotors = new LinkedHashMap<>();
        mMotorMapping = new LinkedHashMap<>();

        mSingleServos = new LinkedHashMap<>();
        mServoMapping = new LinkedHashMap<>();

    }

    public Map<String, MotorComponent>  singleMotors()  { return mSingleMotors; }
    public Map<String, List<String>>    mappingMotors() { return mMotorMapping; }

    public Map<String, ServoComponent>  singleServos()  { return mSingleServos; }
    public Map<String, List<String>>    mappingServos() { return mServoMapping; }

    @Override
    public void                         read(JSONObject reader) {
        super.read(reader);
        try {

            // Read Motors
            if (reader.has(sMotorsKey)) {

                JSONObject motors = reader.getJSONObject(sMotorsKey);
                Iterator<String> keys = motors.keys();
                while (keys.hasNext()) {

                    String key = keys.next();

                    JSONArray array = motors.getJSONArray(key);
                    List<String> mapping = new ArrayList<String>();

                    for(int i_motor = 0; i_motor < array.length(); i_motor ++) {

                        JSONObject single = array.getJSONObject(i_motor);
                        JSONArray singlearray = new JSONArray();
                        singlearray.put(single);
                        String name = single.getString(MotorComponent.sHwMapKey);
                        MotorComponent motor = MotorComponent.factory(name,singlearray, mMap, mLogger);
                        mSingleMotors.put(name,motor);
                        mapping.add(name);

                    }
                    mMotorMapping.put(key,mapping);

                }
            }

            // Read Servos
            if (reader.has(sServosKey)) {

                JSONObject servos = reader.getJSONObject(sServosKey);
                Iterator<String> keys = servos.keys();
                while (keys.hasNext()) {

                    String key = keys.next();

                    JSONArray array = servos.getJSONArray(key);
                    List<String> mapping = new ArrayList<String>();

                    for(int i_servo = 0; i_servo < array.length(); i_servo ++) {

                        JSONObject single = array.getJSONObject(i_servo);
                        JSONArray singlearray = new JSONArray();
                        singlearray.put(single);
                        String name = single.getString(ServoComponent.sHwMapKey);
                        ServoComponent motor = ServoComponent.factory(name,singlearray, mMap, mLogger);
                        mSingleServos.put(name,motor);
                        mapping.add(name);

                    }
                    mServoMapping.put(key,mapping);

                }
            }

        } catch (JSONException e) {
            mLogger.error(e.getMessage());
        }



    }

}
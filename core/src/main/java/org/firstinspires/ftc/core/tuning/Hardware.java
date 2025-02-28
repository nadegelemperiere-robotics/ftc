/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Hardware manager
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.tuning;

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
import org.firstinspires.ftc.core.components.motors.MotorCoupled;
import org.firstinspires.ftc.core.components.servos.ServoComponent;
import org.firstinspires.ftc.core.components.servos.ServoCoupled;

/* Tuning includes */

public class Hardware extends org.firstinspires.ftc.core.robot.Hardware {

    final Map<String, MotorComponent> mSingleMotors;
    final Map<String, List<String>>   mMotorMapping;

    final Map<String, ServoComponent> mSingleServos;
    final Map<String, List<String>>   mServoMapping;

    /**
     * Constructor
     *
     * @param map hardware map to retrieve hardware from
     * @param logger logger to use for trace
     */

    public Hardware(Tuning asker, HardwareMap map, LogManager logger) {
        super(map, logger);

        mSingleMotors = new LinkedHashMap<>();
        mMotorMapping = new LinkedHashMap<>();

        mSingleServos = new LinkedHashMap<>();
        mServoMapping = new LinkedHashMap<>();

    }

    public Map<String, MotorComponent>  singleMotors(Tuning asker)          { return mSingleMotors; }
    public Map<String, List<String>>    mappingMotors(Tuning asker)         { return mMotorMapping; }

    public Map<String, ServoComponent>  singleServos(Tuning asker)          { return mSingleServos; }
    public Map<String, List<String>>    mappingServos(Tuning asker)         { return mServoMapping; }

    /**
     * Extended configuration reading where each coupled servos and motors
     * are added to single servos and motors list as a single servo or motor
     * for separate tuning
     *
     * @param reader JSON object containing configuration to read
     */
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

    /**
     * Extended configuration saving. Tuned configuration for single motors and
     * single servos are saved into the coupled configuration
     *
     */
    public void                         save(Tuning tuning) {

        if(mConfigurationValid) {
            try {

                for (Map.Entry<String, MotorComponent> motor : mMotors.entrySet()) {
                    List<String> mappedSingle = mMotorMapping.get(motor.getKey());
                    JSONObject motors = new JSONObject();
                    if (mappedSingle != null && mappedSingle.size() == 1) {
                        MotorComponent single = mSingleMotors.get(mappedSingle.get(0));
                        if (single != null && single.isConfigured()) {
                            single.write(motors);
                            motor.getValue().read(motors);
                        }
                    } else if (mappedSingle != null && mappedSingle.size() == 2) {
                        JSONObject motor1 = new JSONObject();
                        JSONObject motor2 = new JSONObject();
                        MotorComponent first = mSingleMotors.get(mappedSingle.get(0));
                        MotorComponent second = mSingleMotors.get(mappedSingle.get(1));
                        if (first != null && first.isConfigured()) {
                            first.write(motor1);
                        }
                        if (second != null && second.isConfigured()) {
                            second.write(motor2);
                        }
                        motors.put(MotorCoupled.sFirstKey, motor1);
                        motors.put(MotorCoupled.sSecondKey, motor2);
                        motor.getValue().read(motors);
                    }

                }

                for (Map.Entry<String, ServoComponent> servo : mServos.entrySet()) {
                    List<String> mappedSingle = mServoMapping.get(servo.getKey());
                    JSONObject servos = new JSONObject();
                    if (mappedSingle != null && mappedSingle.size() == 1) {
                        ServoComponent single = mSingleServos.get(mappedSingle.get(0));
                        if (single != null && single.isConfigured()) {
                            single.write(servos);
                            servo.getValue().read(servos);
                        }
                    } else if (mappedSingle != null && mappedSingle.size() == 2) {
                        JSONObject servo1 = new JSONObject();
                        JSONObject servo2 = new JSONObject();
                        ServoComponent first = mSingleServos.get(mappedSingle.get(0));
                        ServoComponent second = mSingleServos.get(mappedSingle.get(1));
                        if (first != null && first.isConfigured()) {
                            first.write(servo1);
                        }
                        if (second != null && second.isConfigured()) {
                            second.write(servo2);
                        }
                        servos.put(ServoCoupled.sFirstKey, servo1);
                        servos.put(ServoCoupled.sSecondKey, servo2);
                        servo.getValue().read(servos);
                    }

                }


            } catch (JSONException e) {
                mLogger.error(e.getMessage());
            }
        }

    }
}
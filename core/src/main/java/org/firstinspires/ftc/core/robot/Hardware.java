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

/* Json includes */
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONArray;

/* Qualcomm includes */
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/* ACME robotics includes */
import com.acmerobotics.roadrunner.ftc.LynxFirmware;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configurable;

/* Components includes */
import org.firstinspires.ftc.core.components.motors.MotorComponent;
import org.firstinspires.ftc.core.components.motors.MotorCoupled;
import org.firstinspires.ftc.core.components.servos.ServoComponent;
import org.firstinspires.ftc.core.components.servos.ServoCoupled;
import org.firstinspires.ftc.core.components.imus.ImuComponent;
import org.firstinspires.ftc.core.components.voltage.VoltageSensorComponent;
import org.firstinspires.ftc.core.components.localizers.LocalizerComponent;

public class Hardware extends HardwareMap implements Configurable {

    static final protected String   sVoltageKey     = "voltage";
    static final protected String   sMotorsKey      = "motors";
    static final protected String   sImusKey        = "imus";
    static final protected String   sServosKey      = "servos";
    static final protected String   sLocalizersKey  = "localizers";

    final protected LogManager                      mLogger;

    protected boolean                               mConfigurationValid;

    final protected HardwareMap                     mMap;

    final protected Map<String, MotorComponent>     mMotors;
    final protected Map<String, ServoComponent>     mServos;
    final protected Map<String, ImuComponent>       mImus;
    final protected Map<String, LocalizerComponent> mLocalizers;
    VoltageSensorComponent                          mVoltageSensor;
    String                                          mVoltageSensorName;

    /**
     * Constructor
     *
     * @param map The controllers hardware map
     * @param logger The logger to report events
     */
    public Hardware(HardwareMap map, LogManager logger) {
        super(null, null);

        mLogger = logger;

        mConfigurationValid = true;

        mMap = map;

        if(mMap != null) {
            LynxFirmware.throwIfModulesAreOutdated(mMap);
            for (LynxModule module : mMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }

        mMotors         = new LinkedHashMap<>();
        mServos         = new LinkedHashMap<>();
        mImus           = new LinkedHashMap<>();
        mLocalizers     = new LinkedHashMap<>();
        mVoltageSensor  = null;
        mVoltageSensorName = "";

    }

    public Map<String,MotorComponent>       motors() { return mMotors; }
    public Map<String,ServoComponent>       servos() { return mServos; }
    public Map<String,ImuComponent>         imus() { return mImus; }
    public Map<String,LocalizerComponent>   localizers() { return mLocalizers; }
    public VoltageSensorComponent           voltageSensor() { return mVoltageSensor; }



    public boolean                          isConfigured() { return mConfigurationValid;}

    // Manage bulkcaching on our own, since it does not work for Imu
    public void                             update() {
        mLogger.info(LogManager.Target.FILE, "start");
        for (Map.Entry<String, ImuComponent> imu : mImus.entrySet()) {
            imu.getValue().update();
        }
        // Call other components at least once to trigger bulk caching
        for (Map.Entry<String, MotorComponent> motor : mMotors.entrySet()) {
            motor.getValue().getCurrentPosition();
        }
        for (Map.Entry<String, ServoComponent> servo : mServos.entrySet()) {
            servo.getValue().getPosition();
        }
        mLogger.info(LogManager.Target.FILE, "stop");
    }

    public void                             read(JSONObject reader) {

        mConfigurationValid = true;
        mVoltageSensor      = null;
        mVoltageSensorName  = "";
        mMotors.clear();
        mImus.clear();
        mServos.clear();
        mLocalizers.clear();

        try {

            if (reader.has(sVoltageKey)) {
                mVoltageSensorName = reader.getString(sVoltageKey);
                mVoltageSensor = VoltageSensorComponent.factory(mVoltageSensorName, reader, mMap, mLogger);
                if(mVoltageSensor == null) {
                    mLogger.warning("Voltage sensor type " + mVoltageSensorName + " not recognized by factory");
                    mConfigurationValid = false;
                }
                else if (!mVoltageSensor.isConfigured()) {
                    mLogger.warning("Voltage sensor type " + mVoltageSensorName + " configuration is invalid");
                    mConfigurationValid = false;
                }
                else {
                    voltageSensor = new DeviceMapping(VoltageSensor.class);
                    voltageSensor.put(mVoltageSensorName, mVoltageSensor);
                }
            }

            // Read Motors
            if (reader.has(sMotorsKey)) {

                JSONObject motors = reader.getJSONObject(sMotorsKey);
                Iterator<String> keys = motors.keys();
                while (keys.hasNext()) {

                    String key = keys.next();

                    MotorComponent motor = MotorComponent.factory(key, motors.getJSONArray(key), mMap, mLogger);
                    if(motor == null) {
                        mLogger.warning("Motor " + key + " not recognized by factory");
                        mConfigurationValid = false;
                    }
                    else if (!motor.isConfigured()) {
                        mLogger.warning("Motor " + key + " configuration is invalid");
                        mConfigurationValid = false;
                    } else { mMotors.put(key, motor); }

                }
            }

            // Read Servos
            if (reader.has(sServosKey)) {

                JSONObject servos = reader.getJSONObject(sServosKey);
                Iterator<String> keys = servos.keys();
                while (keys.hasNext()) {

                    String key = keys.next();

                    ServoComponent servo = ServoComponent.factory(key, servos.getJSONArray(key), mMap, mLogger);
                    if(servo == null) {
                        mLogger.warning("Servo " + key + " not recognized by factory");
                        mConfigurationValid = false;
                    }
                    else if (!servo.isConfigured()) {
                        mLogger.warning("Servo " + key + " configuration is invalid");
                        mConfigurationValid = false;
                    } else { mServos.put(key, servo); }

                }
            }

            // Read Imus
            if (reader.has(sImusKey)) {

                JSONObject imus = reader.getJSONObject(sImusKey);
                Iterator<String> keys = imus.keys();
                while (keys.hasNext()) {

                    String key = keys.next();

                    ImuComponent imu = ImuComponent.factory(key, imus.getJSONObject(key), mMap, mLogger);
                    if(imu == null) {
                        mLogger.warning("Imu " + key + " not recognized by factory");
                        mConfigurationValid = false;
                    }
                    else if (!imu.isConfigured()) {
                        mLogger.warning("Imu " + key + " configuration is invalid");
                        mConfigurationValid = false;
                    } else { mImus.put(key, imu); }

                }
            }

            // Read Odometers
            if (reader.has(sLocalizersKey)) {

                JSONObject odometers = reader.getJSONObject(sLocalizersKey);
                Iterator<String> keys = odometers.keys();
                while (keys.hasNext()) {

                    String key = keys.next();

                    LocalizerComponent localizer = LocalizerComponent.factory(key, odometers.getJSONObject(key), mMap, mMotors, mImus, mLogger);
                    if(localizer == null) {
                        mLogger.warning("Localizer " + key + " not recognized by factory");
                        mConfigurationValid = false;
                    }
                    else if (!localizer.isConfigured()) {
                        mLogger.warning("Localizer " + key + " configuration is invalid");
                        mConfigurationValid = false;
                    } else { mLocalizers.put(key, localizer); }

                }
            }
        } catch (JSONException e) {
            mLogger.error(e.getMessage());
        }
    }

    public void                         write(JSONObject writer) {

        try {

            // Write voltage sensor
            writer.put(sVoltageKey,mVoltageSensorName);

            // Write motors
            JSONObject motors = new JSONObject();
            for (Map.Entry<String, MotorComponent> motor : mMotors.entrySet()) {
                JSONArray array = new JSONArray();
                JSONObject temp = new JSONObject();
                motor.getValue().write(temp);
                if (temp.has(MotorCoupled.sFirstKey)) {
                    array.put(temp.getJSONObject(MotorCoupled.sFirstKey));
                }
                if (temp.has(MotorCoupled.sSecondKey)) {
                    array.put(temp.getJSONObject(MotorCoupled.sSecondKey));
                }
                if (temp.length() != 0 && !temp.has(MotorCoupled.sSecondKey) && !temp.has(MotorCoupled.sSecondKey)) {
                    array.put(temp);
                }
                motors.put(motor.getKey(), array);
            }
            writer.put(sMotorsKey, motors);

            // Write servos
            JSONObject servos = new JSONObject();
            for (Map.Entry<String, ServoComponent> servo : mServos.entrySet()) {
                JSONArray array = new JSONArray();
                JSONObject temp = new JSONObject();
                servo.getValue().write(temp);
                if (temp.has(ServoCoupled.sFirstKey)) {
                    array.put(temp.getJSONObject(ServoCoupled.sFirstKey));
                }
                if (temp.has(ServoCoupled.sSecondKey)) {
                    array.put(temp.getJSONObject(ServoCoupled.sSecondKey));
                }
                if (temp.length() != 0 && !temp.has(ServoCoupled.sSecondKey) && !temp.has(ServoCoupled.sSecondKey)) {
                    array.put(temp);
                }
                servos.put(servo.getKey(), array);
            }
            writer.put(sServosKey, servos);

            // Write imus
            JSONObject imus = new JSONObject();
            for (Map.Entry<String, ImuComponent> imu : mImus.entrySet()) {
                JSONObject temp = new JSONObject();
                imu.getValue().write(temp);
                imus.put(imu.getKey(), temp);
            }
            writer.put(sImusKey, imus);

            // Write odometers
            JSONObject localizers = new JSONObject();
            for (Map.Entry<String, LocalizerComponent> localizer : mLocalizers.entrySet()) {
                JSONObject temp = new JSONObject();
                localizer.getValue().write(temp);
                localizers.put(localizer.getKey(), temp);
            }
            writer.put(sLocalizersKey, localizers);

        } catch (JSONException e) { mLogger.error(e.getMessage()); }
    }

    public String                       logConfigurationHTML()
    {
        StringBuilder result = new StringBuilder();

        result.append("<details style=\"margin-left:10px\">\n");
        result.append("<summary style=\"font-size: 12px; font-weight: 500\"> HARDWARE </summary>\n");
        result.append("<ul>\n");

        // Log voltage sensor
        result.append("<p style=\"margin-left:10px; font-size: 12px; font-weight: 500\"> VOLTAGE : ")
                .append(mVoltageSensorName)
                .append("</p>");

        // Log motors
        result.append("<details style=\"margin-left:10px\">\n");
        result.append("<summary style=\"font-size: 12px; font-weight: 500\"> MOTORS </summary>\n");
        result.append("<ul>\n");
        mMotors.forEach((key, value) -> result.append("<details style=\"margin-left:10px\">\n")
                .append("<summary style=\"font-size: 11px; font-weight: 500\"> ")
                .append(key.toUpperCase())
                .append(" </summary>\n")
                .append("<ul>\n")
                .append(value.logConfigurationHTML())
                .append("</ul>\n")
                .append("</details>\n"));
        result.append("</ul>\n");
        result.append("</details>\n");

        // Log servos
        result.append("<details style=\"margin-left:10px\">\n");
        result.append("<summary style=\"font-size: 12px; font-weight: 500\"> SERVOS </summary>\n");
        result.append("<ul>\n");
        mServos.forEach((key, value) -> result.append("<details style=\"margin-left:10px\">\n")
                    .append("<summary style=\"font-size: 11px; font-weight: 500\"> ")
                    .append(key.toUpperCase())
                    .append(" </summary>\n")
                    .append("<ul>\n")
                    .append(value.logConfigurationHTML())
                    .append("</ul>\n")
                    .append("</details>\n"));
        result.append("</ul>\n");
        result.append("</details>\n");


        // Log imus
        result.append("<details style=\"margin-left:10px\">\n");
        result.append("<summary style=\"font-size: 12px; font-weight: 500\"> IMUS </summary>\n");
        result.append("<ul>\n");
        mImus.forEach((key, value) -> result.append("<details style=\"margin-left:10px\">\n")
                .append("<summary style=\"font-size: 11px; font-weight: 500\"> ")
                .append(key.toUpperCase())
                .append(" </summary>\n")
                .append("<ul>\n")
                .append(value.logConfigurationHTML())
                .append("</ul>\n")
                .append("</details>\n"));
        result.append("</ul>\n");
        result.append("</details>\n");


        // Log odometers
        result.append("<details style=\"margin-left:10px\">\n");
        result.append("<summary style=\"font-size: 12px; font-weight: 500\"> LOCALIZERS </summary>\n");
        result.append("<ul>\n");
        mLocalizers.forEach((key, value) -> result.append("<details style=\"margin-left:10px\">\n")
                    .append("<summary style=\"font-size: 11px; font-weight: 500\"> ")
                    .append(key.toUpperCase())
                    .append(" </summary>\n")
                    .append("<ul>\n")
                    .append(value.logConfigurationHTML())
                    .append("</ul>\n")
                    .append("</details>\n"));
        result.append("</ul>\n");
        result.append("</details>\n");

        result.append("</ul>\n");
        result.append("</details>\n");

        return result.toString();

    }

    public String                       logConfigurationText(String header)
    {
        StringBuilder result = new StringBuilder();

        // Log voltage sensor
        result.append(header)
                .append("> VOLTAGE : ")
                .append(mVoltageSensorName)
                .append("\n");

        // Log motors
        result.append(header)
                .append("> MOTORS\n");

        mMotors.forEach((key, value) -> result.append(header)
                    .append("--> ")
                    .append(key)
                    .append("\n")
                    .append(value.logConfigurationText(header + "----")));

        // Log servos
        result.append(header)
                .append("> SERVOS\n");

        mServos.forEach((key, value) -> result.append(header)
                    .append("--> ")
                    .append(key)
                    .append("\n")
                    .append(value.logConfigurationText(header + "----")));

        // Log imus
        result.append(header)
                .append("> IMUS\n");

        mImus.forEach((key, value) -> result.append(header)
                    .append("--> ")
                    .append(key)
                    .append("\n")
                    .append(value.logConfigurationText(header + "----")));

        // Log odometers
        result.append(header)
                .append("> LOCALIZERS\n");

        mLocalizers.forEach((key, value) -> result.append(header)
                    .append("--> ")
                    .append(key)
                    .append("\n")
                    .append(value.logConfigurationText(header + "----")));

        return result.toString();

    }

    /**
     * Retrieves the (first) device with the indicated name which is also an instance of theindicated class or interface. If no such device is found, an exception is thrown.
     * @param classOrInterface the class or interface indicating the type of the device object to be retrieved
     * @param deviceName name of the device
     * @return Found device
     */
    @Override
    public <T> T get(Class<? extends T> classOrInterface, String deviceName){
        T result = tryGet(classOrInterface, deviceName);
        if (result == null) throw new IllegalArgumentException(
                String.format("No %s named %s is found.", classOrInterface.getName(), deviceName)
        );
        return result;
    }

    /**
     * Retrieves the (first) device with the indicated name which is also an instance of theindicated class or interface. If no such device is found, result is null
     * @param classOrInterface the class or interface indicating the type of the device object to be retrieved
     * @param deviceName name of the device
     * @return found device - null if not found
     */
    @Override
    public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName){

        T result = null;
        if(mConfigurationValid) {
            if ((classOrInterface == DcMotor.class) || (classOrInterface == DcMotorEx.class)) {
                if (mMotors.containsKey(deviceName)) {
                    result = classOrInterface.cast(mMotors.get(deviceName));
                }
            }
            if (classOrInterface == Servo.class) {
                if (mServos.containsKey(deviceName)) {
                    result = classOrInterface.cast(mServos.get(deviceName));
                }
            }
            if (classOrInterface == VoltageSensor.class) {
                result = classOrInterface.cast(mVoltageSensor);
            }
        }

        return result;
    }
}
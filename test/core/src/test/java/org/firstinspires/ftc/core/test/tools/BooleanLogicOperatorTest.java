/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Boolean logic operator test
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.test.tools;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;
import java.io.File;

/* Android includes */
import android.os.Environment;

/* Json includes */
import org.json.JSONObject;

/* Junit 5 includes */
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/* Mockito includes */
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.mockito.junit.jupiter.MockitoExtension;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Configuration includes */
import org.firstinspires.ftc.core.configuration.Configuration;

/* Component Under Test includes */
import org.firstinspires.ftc.core.tools.BooleanLogicOperator;

@ExtendWith(MockitoExtension.class)
public class BooleanLogicOperatorTest {

    private LogManager              mLogger;
    private BooleanLogicOperator    mOperator;
    private Configuration           mConfiguration;

    @BeforeEach
    public void setUp() {
        try (MockedStatic<Environment> mockedEnvironment = Mockito.mockStatic(Environment.class)) {
            mockedEnvironment.when(Environment::getExternalStorageDirectory).thenReturn(new File(getClass().getClassLoader().getResource("results").getFile()));
            mLogger = new LogManager(null, null, "boolean-logic-test",3);
            mLogger.info("Setting it up!");
        }
    }

    @AfterEach
    public void tearDown() {
        mLogger.stop();
    }

    @Test
    public void read() {

        mConfiguration = new Configuration(mLogger);
        mOperator      = new BooleanLogicOperator(mLogger);
        mConfiguration.register("logic", mOperator);
        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/logic-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is valid");

        Map<Integer, JSONObject> values = mOperator.values();
        assertEquals(2, values.size(), "Operator does not have 2 values");


        Map<Integer, Boolean>    data = new LinkedHashMap<>();
        assertDoesNotThrow(() -> {
            for (Map.Entry<Integer, JSONObject> value : values.entrySet()) {
                if(value.getValue().getString("var").equals("A")) { data.put(value.getKey(), false); }
                if(value.getValue().getString("var").equals("B")) { data.put(value.getKey(), true); }
            }
        });
        Boolean result = mOperator.evaluate(data);
        assertFalse(result);

        assertDoesNotThrow(() -> {
            for (Map.Entry<Integer, JSONObject> value : values.entrySet()) {
                if(value.getValue().getString("var").equals("A")) { data.put(value.getKey(), true); }
                if(value.getValue().getString("var").equals("B")) { data.put(value.getKey(), true); }
            }
        });
        result = mOperator.evaluate(data);
        assertTrue(result);
    }

    @Test
    public void write() {

        mConfiguration = new Configuration(mLogger);
        mOperator      = new BooleanLogicOperator(mLogger);
        mConfiguration.register("logic", mOperator);
        mConfiguration.read(getClass().getClassLoader().getResource("data/" + this.getClass().getSimpleName() + "/logic-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is valid");

        mConfiguration.log();

        mConfiguration.write(getClass().getClassLoader().getResource("results").getFile() + "/boolean-logic-test-write.json");

    }


}

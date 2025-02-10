/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Configuration management test class
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.configuration;

/* System includes */
import java.io.File;

/* Android includes */
import android.os.Environment;

/* Junit 5 includes */
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/* Mockito includes */
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.mockito.junit.jupiter.MockitoExtension;

/* Json includes */
import org.json.JSONObject;
import org.json.JSONException;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;

/* Component Under Test includes */
import org.firstinspires.ftc.core.configuration.Configuration;
import org.firstinspires.ftc.core.configuration.Configurable;

@ExtendWith(MockitoExtension.class)
public class ConfigurationTest {

    private LogManager mLogger;
    private Configuration mConfiguration;

    @BeforeEach
    public void setUp() {
        try (MockedStatic<Environment> mockedEnvironment = Mockito.mockStatic(Environment.class)) {
            mockedEnvironment.when(Environment::getExternalStorageDirectory).thenReturn(new File(getClass().getClassLoader().getResource("results").getFile()));
            mLogger = new LogManager(null, null, "configuration-test");
            mLogger.info("Setting it up!");
        }
    }

    @Test
    public void read() {
        mConfiguration = new Configuration(mLogger);

        ConfigurableTest config1 = new ConfigurableTest();
        ConfigurableTest config2 = new ConfigurableTest();
        ConfigurableTest config3 = new ConfigurableTest();
        ConfigurableTest config4 = new ConfigurableTest();

        mConfiguration.register("array1[0][0]", config1);
        mConfiguration.register("array1[0][1]", config2);
        mConfiguration.register("data1", config3);
        mConfiguration.register("data2.data1[0]", config4);

        mConfiguration.read(getClass().getClassLoader().getResource("data/configuration/valid-test-1.json").getFile());

        assertTrue(mConfiguration.isValid(), "Configuration is invalid");

        assertTrue(config1.isConfigured(),   "Configuration 1 is invalid");
        assertEquals(config1.mData1,2,       "Configuration 1 data1 is not 2");
        assertEquals(config1.mData2,3,       "Configuration 1 data2 is not 3");
        assertTrue(config2.isConfigured(),   "Configuration 2 is invalid");
        assertEquals(config2.mData1,4,       "Configuration 2 data1 is not 4");
        assertEquals(config2.mData2,5,       "Configuration 2 data2 is not 5");
        assertTrue(config3.isConfigured(),   "Configuration 3 is invalid");
        assertEquals(config3.mData1,10,      "Configuration 3 data1 is not 10");
        assertEquals(config3.mData2,11,      "Configuration 3 data2 is not 11");
        assertTrue(config4.isConfigured(),   "Configuration 4 is invalid");
        assertEquals(config4.mData1,12,      "Configuration 4 data1 is not 12");
        assertEquals(config4.mData2,13,      "Configuration 4 data2 is not 13");

    }

    @Test
    public void invalid_request() {
        mConfiguration = new Configuration(mLogger);

        ConfigurableTest config1 = new ConfigurableTest();
        ConfigurableTest config2 = new ConfigurableTest();
        ConfigurableTest config3 = new ConfigurableTest();
        ConfigurableTest config4 = new ConfigurableTest();
        ConfigurableTest config5 = new ConfigurableTest();
        ConfigurableTest config6 = new ConfigurableTest();

        mConfiguration.register("array1[2][0]", config1);
        mConfiguration.register("array1[0][2]", config2);
        mConfiguration.register("array1[0].data1", config3);
        mConfiguration.register("data3", config4);
        mConfiguration.register("data2.data2[0]", config5);
        mConfiguration.register("array1[1][1]", config6);

        mConfiguration.read(getClass().getClassLoader().getResource("data/configuration/valid-test-1.json").getFile());

        assertFalse(mConfiguration.isValid(), "Configuration valid");

        assertFalse(config1.isConfigured(), "Configuration 1 is valid");
        assertEquals(config1.mData1,0,      "Configuration 1 data1 has changed");
        assertEquals(config1.mData2,0,      "Configuration 1 data2 has changed");
        assertFalse(config2.isConfigured(), "Configuration 2 is valid");
        assertEquals(config2.mData1,0,      "Configuration 2 data1 has changed");
        assertEquals(config2.mData2,0,      "Configuration 2 data2 has changed");
        assertFalse(config3.isConfigured(), "Configuration 3 is valid");
        assertEquals(config3.mData1,0,      "Configuration 3 data1 has changed");
        assertEquals(config3.mData2,0,      "Configuration 3 data2 has changed");
        assertFalse(config4.isConfigured(), "Configuration 4 is valid");
        assertEquals(config4.mData1,0,      "Configuration 4 data1 has changed");
        assertEquals(config4.mData2,0,      "Configuration 4 data2 has changed");
        assertFalse(config5.isConfigured(), "Configuration 5 is valid");
        assertEquals(config5.mData1,0,      "Configuration 5 data1 has changed");
        assertEquals(config5.mData2,0,      "Configuration 5 data2 has changed");
        assertTrue(config6.isConfigured(),  "Configuration 6 is invalid");
        assertEquals(config6.mData1,8,      "Configuration 6 data1 is not 8");
        assertEquals(config6.mData2,9,      "Configuration 6 data1 is not 9");

    }

    @Test
    public void write() {
        mConfiguration = new Configuration(mLogger);

        ConfigurableTest config1 = new ConfigurableTest();
        ConfigurableTest config2 = new ConfigurableTest();
        ConfigurableTest config3 = new ConfigurableTest();
        ConfigurableTest config4 = new ConfigurableTest();
        ConfigurableTest config5 = new ConfigurableTest();
        ConfigurableTest config6 = new ConfigurableTest();
        ConfigurableTest config7 = new ConfigurableTest();
        ConfigurableTest config8 = new ConfigurableTest();
        ConfigurableTest config9 = new ConfigurableTest();
        ConfigurableTest config10 = new ConfigurableTest();

        mConfiguration.register("array1[0][0]", config1);
        mConfiguration.register("array1[0][1]", config2);
        mConfiguration.register("data1", config3);
        mConfiguration.register("data2.data1[0]", config4);
        mConfiguration.register("array1[2][0]", config5);
        mConfiguration.register("array1[0][2]", config6);
        mConfiguration.register("array1[0].data1", config7);
        mConfiguration.register("data3", config8);
        mConfiguration.register("data2.data2[0]", config9);
        mConfiguration.register("array1[1][1]", config10);

        mConfiguration.read(getClass().getClassLoader().getResource("data/configuration/valid-test-1.json").getFile());

        assertFalse(mConfiguration.isValid(), "Configuration is valid");

        mConfiguration.write(getClass().getClassLoader().getResource("results").getFile() + "/write.json");


    }
}


class ConfigurableTest implements Configurable {
    public double      mData1;
    public double      mData2;
    boolean            mConfigurationValid;

    public ConfigurableTest() {
        mData1 = 0;
        mData2 = 0;
        mConfigurationValid = false;
    }

    public void read(JSONObject reader) {
        mConfigurationValid = true;
        try {
            if (reader.has("data1")) { mData1 = reader.getDouble("data1"); }
            else { mConfigurationValid = false; }
            if (reader.has("data2")) { mData2 = reader.getDouble("data2"); }
            else { mConfigurationValid = false; }
        }
        catch(JSONException e ) { mConfigurationValid = false; }
    }

    public void write(JSONObject writer) {
        if(mConfigurationValid) {
            try {
                writer.put("data1",mData1);
                writer.put("data2",mData2);
            }
            catch(JSONException e ) {  }
        }
    }

    public boolean isConfigured() { return mConfigurationValid; }

    public String logConfiguration() {

        String result = "";
        result += "<li style=\"padding-left:10px;font-size: 13px\"> data1 : " +
                mData1 +
                "</li>";
        result += "<li style=\"padding-left:10px;font-size: 13px\"> data2 : " +
                mData2 +
                "</li>";
        return result;

    }
}


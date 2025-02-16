/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Sequencer test class
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.test.orchestration;

/* System includes */
import java.io.File;
import java.util.List;
import java.util.ArrayList;

/* Android includes */
import android.os.Environment;

/* Junit 5 includes */
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/* Mockito includes */
import org.mockito.MockedStatic;
import org.mockito.Mockito;
import org.mockito.junit.jupiter.MockitoExtension;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;
import org.firstinspires.ftc.core.tools.Condition;
import org.firstinspires.ftc.core.tools.Timer;

/* Component Under Test includes */
import org.firstinspires.ftc.core.orchestration.engine.Sequencer;
import org.firstinspires.ftc.core.orchestration.engine.Task;


@ExtendWith(MockitoExtension.class)
public class SequencerTest {

    private LogManager      mLogger;
    private Sequencer       mSequencer;
    private List<String>    mExecution;
    private List<Long>      mStartTime;
    private Timer           mTimer;

    @BeforeEach
    public void setUp() {
        try (MockedStatic<Environment> mockedEnvironment = Mockito.mockStatic(Environment.class)) {
            mockedEnvironment.when(Environment::getExternalStorageDirectory).thenReturn(new File(getClass().getClassLoader().getResource("results").getFile()));
            mLogger = new LogManager(null, null, "sequencer-test",3);
            mLogger.level(LogManager.Severity.INFO);
            mLogger.info("Setting it up!");
        }
    }

    @Test
    public void nominal() {

        mSequencer  = new Sequencer(mLogger);
        mExecution  = new ArrayList<>();
        mStartTime  = new ArrayList<>();
        mTimer      = new Timer(mLogger);

        mSequencer.sequence(
                "NOMINAL",
                new Task(
                        "Task1",
                        () -> {
                            mStartTime.add(System.nanoTime());
                            mExecution.add("Task1");
                            mTimer.arm(200);
                        },
                        new Condition(() -> !mTimer.isArmed())
                ),
                new Task(
                        "Task2",
                        () -> {
                            mStartTime.add(System.nanoTime());
                            mExecution.add("Task2");
                            mTimer.arm(500);
                        },
                        new Condition(() -> !mTimer.isArmed())
                ),
                new Task(
                        "Task3",
                        () -> {
                            mStartTime.add(System.nanoTime());
                            mExecution.add("Task3");
                            mTimer.arm(1000);
                        },
                        new Condition(() -> !mTimer.isArmed())
                )
        );
        mSequencer.run();
        while(!mSequencer.hasFinished()) {
            try {
                Thread.sleep(10);
            }
            catch(InterruptedException ignored) {}
        }
        mStartTime.add(System.nanoTime());

        assertEquals(3,mExecution.size(),"3 tasks shall have been executed");
        assertEquals("Task1", mExecution.get(0), "Task 1 shall execute first");
        assertEquals("Task2", mExecution.get(1), "Task 2 shall execute first");
        assertEquals("Task3", mExecution.get(2), "Task 3 shall execute first");

        assertEquals(4,mStartTime.size(),"4 times should have been recorded");
        double duration = (double)(mStartTime.get(1) - mStartTime.get(0)) / 1_000_000;
        mLogger.info("Task1 lasted " + duration);
        assertTrue(Math.abs(duration - 200) < 15,"Task 1 shall have lasted 200 ms ");
        duration = (double)(mStartTime.get(2) - mStartTime.get(1)) / 1_000_000;
        mLogger.info("Task2 lasted " + duration);
        assertTrue(Math.abs(duration - 500) < 15,"Task 2 shall have lasted 500 ms ");
        duration = (double)(mStartTime.get(3) - mStartTime.get(2)) / 1_000_000;
        mLogger.info("Task3 lasted " + duration);
        assertTrue(Math.abs(duration - 1000) < 15,"Task 3 shall have lasted 1000 ms ");

    }
}


/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Scheduler for a list of tasks
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.engine;

/* System includes */
import java.util.Collections;
import java.util.LinkedList;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;


public class Sequencer {

    LogManager          mLogger;
    
    String              mName;
    
    LinkedList<Task>    mTasks;

    /**
     * Constructor
     *
     * @param logger A logger to log sequence
     */
    public Sequencer(LogManager logger) {
        mLogger = logger;
        mTasks  = new LinkedList<>();
    }

    /**
     * Defines the list of task to sequence
     *
     * @param tasks The sequence of tasks to perform
     */
    public  void sequence(Task... tasks)
    {
        this.sequence("",tasks);
    }

    /**
     * Defines the list of task to sequence
     *
     * @param name The name of the task sequence
     * @param tasks The sequence of tasks to perform
     */
    public  void sequence(String name, Task... tasks)
    {
        if(this.hasFinished()) {
            mName   = name;
            Collections.addAll(mTasks, tasks);
        }
    }

    /**
     * Starts the task sequence
     */
    public void run() {

        if(!mTasks.isEmpty()) {
            Task current = mTasks.getFirst();
            if(!mName.isEmpty()) { mLogger.metric(mName, current.name()); }
            current.run();
        }

    }

    /**
     * Check if sequence is over and manage task change - shall be called periodically
     *
     * @return true if the task sequence is over
     */
    public boolean hasFinished() {

        if(!mTasks.isEmpty()) {

            Task current = mTasks.getFirst();
            if (current.hasFinished()) {

                mTasks.remove(current);

                if(!mTasks.isEmpty()) {
                    current = mTasks.getFirst();
                    if(!mName.isEmpty()) { mLogger.metric(mName, current.name()); }
                    current.run();
                }

            }
        }

        return mTasks.isEmpty();
    }


}
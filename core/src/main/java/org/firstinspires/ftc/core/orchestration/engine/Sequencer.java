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

    final LogManager        mLogger;
    
    String                  mName;

    boolean                 mHasFinished;
    
    final LinkedList<Task>  mTasks;

    /**
     * Constructor
     *
     * @param logger A logger to log sequence
     */
    public Sequencer(LogManager logger) {
        mLogger = logger;
        mHasFinished = true;
        mTasks  = new LinkedList<>();
    }

    /**
     * Defines the list of task to sequence
     *
     * @param tasks The sequence of tasks to perform
     */
    public void                         sequence(Task... tasks)
    {
        this.sequence("",tasks);
    }

    /**
     * Defines the list of task to sequence
     *
     * @param name The name of the task sequence
     * @param tasks The sequence of tasks to perform
     */
    public void                         sequence(String name, Task... tasks)
    {
        if(mHasFinished) {
            mName   = name;
            Collections.addAll(mTasks, tasks);
        }
        else {
            mLogger.warning("Forgot sequence " + name + " because still busy with sequence " + mName);
        }
    }

    /**
     * Starts the task sequence
     */
    public void                         run() {

        if(mHasFinished && !mTasks.isEmpty()) {
            mHasFinished = false;
            Task current = mTasks.getFirst();

            if(!mName.isEmpty()) { mLogger.metric(mName, "Start"); }
            if(!mName.isEmpty()) { mLogger.metric(mName, current.name()); }
            current.run();
        }

    }

    /**
     * Manage task change - shall be called periodically
     */
    public void                         update() {

        if(!mTasks.isEmpty()) {

            Task current = mTasks.getFirst();
            if (current.hasFinished()) {

                mTasks.remove(current);

                if(!mTasks.isEmpty()) {
                    current = mTasks.getFirst();
                    if(!mName.isEmpty()) { mLogger.metric(mName, current.name()); }
                    current.run();
                }
                else {
                    if(!mName.isEmpty()) { mLogger.metric(mName, "Stop"); }
                }

            }
        }

        mHasFinished = mTasks.isEmpty();

    }

    /**
     * Check if sequence is over and manage task change - shall be called periodically
     *
     * @return true if the task sequence is over
     */
    public boolean hasFinished() { return mHasFinished; }


}
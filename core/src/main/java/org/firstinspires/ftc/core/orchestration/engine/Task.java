/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Scheduler for a list of tasks
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.engine;

/* Tools includes */
import org.firstinspires.ftc.core.tools.Condition;


public class Task {

    final Condition       mCondition;
    final Runnable        mAction;
    final String          mName;

    /**
     * Constructor
     *
     * @param name The name of the task
     * @param action The action to perform during the task
     * @param condition The condition to decide the task is over
     */
    public Task(String name, Runnable action, Condition condition) {
        mCondition  = condition;
        mAction     = action;
        mName       = name;
    }

    /**
     * Constructor
     *
     * @param action The action to perform during the task
     * @param condition The condition to decide the task is over
     */
    public Task(Runnable action, Condition condition) {
        mCondition  = condition;
        mAction     = action;
        mName       = "";
    }


    /**
     * Name accessor
     *
     * @return The name of the task
     */
    public String   name()          { return mName; }

    /**
     * Task start function
     */
    public void     run()           { mAction.run(); }

    /**
     * Check if task is over
     *
     * @return true if the task is over
     */
    public boolean  hasFinished()   { return mCondition.evaluate(); }

}
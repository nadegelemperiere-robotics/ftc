/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Class mapping controller inputs to robot behavior
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.tools;

/* System includes */
import java.util.function.BooleanSupplier;
import java.util.List;

public class Condition {

    /**
     * AND operator
     *
     * @param conditions a list of input conditions
     */
    public  static Condition and(Condition... conditions) {
        return new Condition(() -> {
            for (Condition condition : conditions) {
                if (!condition.mLogic.getAsBoolean()) {
                    return false;
                }
            }
            return true;
        });
    }
    /**
     * AND operator
     *
     * @param conditions a list of input conditions
     */
    public  static Condition and(List<Condition> conditions) {
        return new Condition(() -> {
            for (Condition condition : conditions) {
                if (!condition.mLogic.getAsBoolean()) {
                    return false;
                }
            }
            return true;
        });
    }

    /**
     * NAND operator
     *
     * @param conditions a list of input conditions
     */
    public  static Condition nand(Condition... conditions) {
        return new Condition(() -> {
            for (Condition condition : conditions) {
                if (!condition.mLogic.getAsBoolean()) {
                    return true;
                }
            }
            return false;
        });
    }
    /**
     * NAND operator
     *
     * @param conditions a list of input conditions
     */
    public  static Condition nand(List<Condition> conditions) {
        return new Condition(() -> {
            for (Condition condition : conditions) {
                if (!condition.mLogic.getAsBoolean()) {
                    return true;
                }
            }
            return false;
        });
    }

    /**
     * OR operator
     *
     * @param conditions a list of input conditions
     */
    public  static Condition or(Condition... conditions) {
        return new Condition(() -> {
            for (Condition condition : conditions) {
                if (condition.mLogic.getAsBoolean()) {
                    return true;
                }
            }
            return false;
        });
    }
    /**
     * OR operator
     *
     * @param conditions a list of input conditions
     */
    public  static Condition or(List<Condition> conditions) {
        return new Condition(() -> {
            for (Condition condition : conditions) {
                if (condition.mLogic.getAsBoolean()) {
                    return true;
                }
            }
            return false;
        });
    }

    /**
     * NOR operator
     *
     * @param conditions a list of input conditions
     */
    public  static Condition nor(Condition... conditions) {
        return new Condition(() -> {
            for (Condition condition : conditions) {
                if (condition.mLogic.getAsBoolean()) {
                    return false;
                }
            }
            return true;
        });
    }
    /**
     * NOR operator
     *
     * @param conditions a list of input conditions
     */
    public  static Condition nor(List<Condition> conditions) {
        return new Condition(() -> {
            for (Condition condition : conditions) {
                if (condition.mLogic.getAsBoolean()) {
                    return false;
                }
            }
            return true;
        });
    }

    /**
     * XOR operator
     *
     * @param conditions a list of input conditions
     */
    public  static Condition xor(Condition... conditions) {
        return new Condition(() -> {
            int numberTrue = 0;
            for (Condition condition : conditions) {
                if (condition.mLogic.getAsBoolean()) {
                    numberTrue ++;
                }
            }
            return ((numberTrue % 2) == 1);
        });
    }
    /**
     * XOR operator
     *
     * @param conditions a list of input conditions
     */
    public  static Condition xor(List<Condition> conditions) {
        return new Condition(() -> {
            int numberTrue = 0;
            for (Condition condition : conditions) {
                if (condition.mLogic.getAsBoolean()) {
                    numberTrue ++;
                }
            }
            return ((numberTrue % 2) == 1);
        });
    }

    /**
     * NOT operator
     *
     * @param condition an input conditions
     */
    public  static Condition not(Condition condition) {
        return new Condition(() -> !condition.mLogic.getAsBoolean());
    }

    final BooleanSupplier mLogic;

    /**
     * Constructor
     *
     * @param supplier boolean supplier to evaluate the condition
     */
    public  Condition(BooleanSupplier supplier)  {
        mLogic = supplier;
    }

    /**
     * Condition evaluation
     *
     * @return true if the condition is met
     */
    public boolean evaluate() {
        return mLogic.getAsBoolean();
    }




}

/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Class mapping controller inputs to robot behavior
   ------------------------------------------------------- */

package org.firstinspires.ftc.core.orchestration.controller;

/* System includes */
import java.util.function.BooleanSupplier;
import java.util.List;

public class Condition {

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

    public  static Condition not(Condition condition) {
        return new Condition(() -> !condition.mLogic.getAsBoolean());
    }

    BooleanSupplier mLogic;

    public  Condition(BooleanSupplier supplier)  {
        mLogic = supplier;
    }

    public boolean evaluate() {
        return mLogic.getAsBoolean();
    }




}

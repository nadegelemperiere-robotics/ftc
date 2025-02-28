/* -------------------------------------------------------
   Copyright (c) [2025] Nadege LEMPERIERE
   All rights reserved
   -------------------------------------------------------
   Intake subsystem
   ------------------------------------------------------- */

package org.firstinspires.ftc.intothedeep.v1.subsystems;

/* JSON includes */
import org.json.JSONException;
import org.json.JSONObject;

/* Tools includes */
import org.firstinspires.ftc.core.tools.LogManager;
import org.firstinspires.ftc.core.tools.Condition;

/* Subsystem includes */
import org.firstinspires.ftc.core.subsystems.Subsystem;
import org.firstinspires.ftc.core.subsystems.Actuator;
import org.firstinspires.ftc.core.subsystems.ToggleActuator;

/* Robot includes */
import org.firstinspires.ftc.core.robot.Hardware;

/* Orchestration includes */
import org.firstinspires.ftc.core.orchestration.engine.Sequencer;
import org.firstinspires.ftc.core.orchestration.engine.Task;


public class OuttakeArm implements Subsystem {

    static final String sClawKey   = "claw";
    static final String sWristKey  = "wrist";
    static final String sElbowKey  = "elbow";

    public enum Position {
        NONE,
        INIT,
        TRANSFER,
        DROP,
        SPECIMEN_BEHIND,
        SPECIMEN_AHEAD,
        VERTICAL
    }


    final LogManager    mLogger;

    boolean             mConfigurationValid;

    final String        mName;

    final Sequencer     mSequencer;
    Position            mPosition;

    final Hardware      mHardware;
    ToggleActuator      mClaw;
    ToggleActuator      mWrist;
    Actuator            mElbow;

    /**
     * Constructor
     *
     * @param hardware The robot current hardware
     * @param logger The logger to report events
     */
    public OuttakeArm(String name, Hardware hardware, LogManager logger) {

        mLogger             = logger;

        mConfigurationValid = false;

        mName               = name;

        mSequencer          = new Sequencer(mLogger);
        mPosition           = Position.NONE;

        mHardware           = hardware;
        mClaw               = null;
        mWrist              = null;
        mElbow              = null;

    }

    /**
     * Move arm into a given reference position
     *
     * @param position position to reach
     */
    public void                         position(Position position) {

        if(mConfigurationValid) {

            switch (position) {

                case INIT :
                    mPosition = position;
                    mSequencer.sequence(
                            new Task(
                                    () -> {
                                        mClaw.position("closed",0,500);
                                        mWrist.position("0",0,500);
                                        mElbow.position("off",0,500);
                                    },
                                    Condition.and(
                                            new Condition(() -> mClaw.hasFinished()),
                                            new Condition(() -> mElbow.hasFinished()),
                                            new Condition(() -> mWrist.hasFinished())
                                    )
                            )
                    );
                    mSequencer.run();
                    break;
                case TRANSFER :
                    mPosition = position;
                    mSequencer.sequence(
                            new Task(
                                    () -> {
                                        mClaw.position("open",0,500);
                                        mWrist.position("0",0,500);
                                        mElbow.position("transfer",0,500);
                                    },
                                    Condition.and(
                                            new Condition(() -> mClaw.hasFinished()),
                                            new Condition(() -> mElbow.hasFinished()),
                                            new Condition(() -> mWrist.hasFinished())
                                    )
                            )
                    );
                    mSequencer.run();
                    break;
                case DROP :
                    mPosition = position;
                    mSequencer.sequence(
                            new Task(
                                    () -> {
                                        mClaw.position("closed",0,500);
                                        mWrist.position("0",0,500);
                                        mElbow.position("drop-sample",0,500);
                                    },
                                    Condition.and(
                                            new Condition(() -> mClaw.hasFinished()),
                                            new Condition(() -> mElbow.hasFinished()),
                                            new Condition(() -> mWrist.hasFinished())
                                    )
                            )
                    );
                    mSequencer.run();
                    break;
                case SPECIMEN_BEHIND :
                    mPosition = position;
                    mSequencer.sequence(
                            new Task(
                                    () -> {
                                        mWrist.position("0",0,500);
                                        mElbow.position("autonomous-specimen-behind",0,500);
                                    },
                                    Condition.and(
                                            new Condition(() -> mClaw.hasFinished()),
                                            new Condition(() -> mElbow.hasFinished()),
                                            new Condition(() -> mWrist.hasFinished())
                                    )
                            )
                    );
                    mSequencer.run();
                    break;
                case SPECIMEN_AHEAD :
                    mPosition = position;
                    mSequencer.sequence(
                            new Task(
                                    () -> {
                                        mClaw.position("closed",0,500);
                                        mWrist.position("0",0,500);
                                        mElbow.position("autonomous-specimen-ahead",0,500);
                                    },
                                    Condition.and(
                                            new Condition(() -> mClaw.hasFinished()),
                                            new Condition(() -> mElbow.hasFinished()),
                                            new Condition(() -> mWrist.hasFinished())
                                    )
                            )
                    );
                    mSequencer.run();
                    break;
                case VERTICAL :
                    mPosition = position;
                    mSequencer.sequence(
                            new Task(
                                    () -> {
                                        mClaw.position("closed",0,500);
                                        mWrist.position("0",0,500);
                                        mElbow.position("vertical",0,500);
                                    },
                                    Condition.and(
                                            new Condition(() -> mClaw.hasFinished()),
                                            new Condition(() -> mElbow.hasFinished()),
                                            new Condition(() -> mWrist.hasFinished())
                                    )
                            )
                    );
                    mSequencer.run();
                    break;

            }
        }

    }

    /** Move arm into a given direction
     *
     * @param direction up or down depending on the direction
     */
    public void                         move( String direction ) {

        if(mConfigurationValid) {
            switch(direction) {
                case "up":
                    switch (mPosition) {
                        case INIT:
                        case TRANSFER:
                            this.position(Position.DROP); break;
                    }
                    break;
                case "down":
                    switch (mPosition) {
                        case INIT:
                        case DROP:
                            this.position(Position.TRANSFER); break;
                    }
                    break;
            }
        }
    }

    /**
     * Close the claw firmly to make specimen turn around the bar when released
     */
    public void                         ultraclose() {
        if(mConfigurationValid) {
            mSequencer.sequence(
                    new Task(
                            () -> mClaw.position("ultraclosed",0,500),
                            new Condition(() -> mClaw.hasFinished())
                    )
            );
            mSequencer.run();
        }
    }

    /**
     * If claw opened, close it. If claw closed, open it and move away from the
     * basket to avoid hurting it after releasing a sample
     */
    public void                         toogleClaw() {
        if(mConfigurationValid) {
            if(mClaw.position().equals("open")) {
                mSequencer.sequence(
                        new Task(
                                () -> mClaw.toggle(500),
                                new Condition(() -> mClaw.hasFinished())
                        )
                );
                mSequencer.run();
            }
            else if(mClaw.position().equals("closed")) {
                mPosition = Position.INIT;
                mSequencer.sequence(
                        "SAFE OPEN CLAW",
                        new Task(
                                "Open",
                                () -> mClaw.toggle(500),
                                new Condition(() -> mClaw.hasFinished())
                        ),
                        new Task(
                                "Away from basket",
                                () -> {
                                    mWrist.position("0",0,200);
                                    mElbow.position("off",0,200);
                                },
                                Condition.and(
                                        new Condition(() -> mWrist.hasFinished()),
                                        new Condition(() -> mElbow.hasFinished())
                                )
                        )
                );
                mSequencer.run();
            }
        }
    }

    /**
     * Update periodically intake status
     */
    public void                         update() {

        if(mConfigurationValid) {
            mClaw.update();
            mWrist.update();
            mElbow.update();
            mSequencer.update();
        }
    }

    /**
     * Check if the intake has reached his position
     *
     * @return true if the position was reached, false otherwise
     */
    public boolean                      hasFinished() { return mSequencer.hasFinished(); }

    /**
     * Manage peristence between op modes
     */
    public void                         persist() {
        mClaw.persist();
        mElbow.persist();
        mWrist.persist();
    }

    /**
     * Determines if the actuator subsystem is configured correctly.
     *
     * @return True if the component is configured, false otherwise.
     */
    public boolean                      isConfigured() { return mConfigurationValid; }

    /**
     * Reads and applies the intake configuration from a JSON object.
     *
     * @param reader The JSON object containing configuration settings.
     */
    public void                         read(JSONObject reader) {

        mClaw               = null;
        mWrist              = null;
        mElbow              = null;
        mConfigurationValid = true;

        try {
            if(reader.has(sClawKey)) {
                JSONObject object = reader.getJSONObject(sClawKey);
                Subsystem subsystem = Subsystem.factory(mName + "-" + sClawKey,object,mHardware,mLogger);
                if(subsystem != null) {
                    if (subsystem instanceof ToggleActuator) { mClaw = (ToggleActuator) subsystem; }
                    else { mLogger.error("Claw subsystem is not of type ToggleActuator");}
                }
            }
            if(reader.has(sWristKey)) {
                JSONObject object = reader.getJSONObject(sWristKey);
                Subsystem subsystem = Subsystem.factory(mName + "-" + sWristKey,object,mHardware,mLogger);
                if(subsystem != null) {
                    if(subsystem instanceof ToggleActuator) { mWrist = (ToggleActuator)subsystem; }
                    else { mLogger.error("Wrist subsystem is not of type ToggleActuator");}
                }
            }
            if(reader.has(sElbowKey)) {
                JSONObject object = reader.getJSONObject(sElbowKey);
                Subsystem subsystem =  Subsystem.factory(mName + "-" + sElbowKey,object,mHardware,mLogger);
                if(subsystem != null) {
                    if(subsystem instanceof Actuator) { mElbow = (Actuator)subsystem; }
                    else { mLogger.error("Elbow subsystem is not of type eActuator");}
                }
            }

        } catch(JSONException e) {
            mLogger.error(e.getMessage());
        }

        if(mClaw == null || !mClaw.isConfigured()) {
            mLogger.error("Intake claw configuration failed");
            mConfigurationValid = false;
        }
        if(mWrist == null || !mWrist.isConfigured()) {
            mLogger.error("Intake wrist configuration failed");
            mConfigurationValid = false;
        }
        if(mElbow == null || !mElbow.isConfigured()) {
            mLogger.error("Intake elbow configuration failed");
            mConfigurationValid = false;
        }

    }

    /**
     * Writes the current intake configuration to a JSON object.
     *
     * @param writer The JSON object to store the configuration settings.
     */
    public void                         write(JSONObject writer) {

        if(mConfigurationValid) {

            try {

                writer.put(sTypeKey, "outtake-arm");

                JSONObject claw = new JSONObject();
                mClaw.write(claw);
                writer.put(sClawKey, claw);

                JSONObject wrist = new JSONObject();
                mWrist.write(wrist);
                writer.put(sWristKey, wrist);

                JSONObject elbow = new JSONObject();
                mElbow.write(elbow);
                writer.put(sElbowKey, elbow);


            } catch (JSONException e) {
                mLogger.error(e.getMessage());
            }
        }
    }

    /**
     * Generates an HTML representation of the actuator configuration for logging purposes.
     *
     * @return A string containing the HTML-formatted actuator configuration.
     */
    @Override
    public String                       logConfigurationHTML() {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {

            // Log claw
            result.append("<details style=\"margin-left:10px\">\n");
            result.append("<summary style=\"font-size: 10px; font-weight: 500\"> CLAW </summary>\n");
            result.append("<ul>\n");
            result.append(mClaw.logConfigurationHTML());
            result.append("</ul>\n");
            result.append("</details>\n");

            // Log wrist
            result.append("<details style=\"margin-left:10px\">\n");
            result.append("<summary style=\"font-size: 10px; font-weight: 500\"> WRIST </summary>\n");
            result.append("<ul>\n");
            result.append(mWrist.logConfigurationHTML());
            result.append("</ul>\n");
            result.append("</details>\n");

            // Log elbow
            result.append("<details style=\"margin-left:10px\">\n");
            result.append("<summary style=\"font-size: 10px; font-weight: 500\"> ELBOW </summary>\n");
            result.append("<ul>\n");
            result.append(mElbow.logConfigurationHTML());
            result.append("</ul>\n");
            result.append("</details>\n");

        }

        return result.toString();

    }

    /**
     * Generates a text-based representation of the actuator configuration for logging.
     *
     * @param header A string to prepend to the configuration log.
     * @return A string containing the formatted actuator configuration details.
     */
    @Override
    public String                       logConfigurationText(String header) {

        StringBuilder result = new StringBuilder();

        if(mConfigurationValid) {

            // Log claw
            result.append(header)
                    .append("> CLAW\n");
            result.append(mClaw.logConfigurationText(header + "--"));

            // Log wrist
            result.append(header)
                    .append("> WRIST\n");
            result.append(mWrist.logConfigurationText(header + "--"));

            // Log elbow
            result.append(header)
                    .append("> ELBOW\n");
            result.append(mElbow.logConfigurationText(header + "--"));


        }

        return result.toString();

    }

    /**
     * Current state logging function
     */
    @Override
    public void                         log() {
        if(this.hasFinished()) { mLogger.info(mName + " : pos = " + mPosition); }
        else { mLogger.info(mName + " : pos > " + mPosition); }
        mElbow.log();
        mWrist.log();
        mClaw.log();
    }


}

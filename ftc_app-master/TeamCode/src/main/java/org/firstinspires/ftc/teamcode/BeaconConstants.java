package org.firstinspires.ftc.teamcode;

/**
 * Created by sarabranham on 11/19/16.
 */

public interface BeaconConstants {
    // Alignment problem  - missed line completely
    // They failed, we did fine
    // Missed board in middle (turn angle??)
    // Dropped ball
    // they stopped in our path, robot fell
    // FINALS
    //

    public static final boolean RED_TEAM = false;
    public static final boolean BALL_ONLY = false;
    public static final boolean SECOND_BEACON = false;
    public static final boolean FOURTH_SQUARE = false;
    public static final boolean RAMP = true;
    public static final double INIT_SLEEP_TIME = 0.1;
    public static final double START_SERVO_TIME = 3;
    public static final double LINE_FORWARD_POWER = 0.7; // 0.5
    public static final double INIT_DRIVE_DISTANCE = 50;
    public static final double LINE_SLOW_POWER = 0.17; // 0.1
    public static final double ROTATE_MAX =  0.3;
    public static final double ROTATE_MIN = 0.1;
    public static final double ALIGN_POWER = 0.16; // 0.15
    public static final double ODS_WHITE_THRESHOLD = 1; //front reads white as 2, ods reads it as 1
    public static final double ODS_GREY_THRESHOLD = 0.5;
    public static final double FRONT_WHITE_THRESHOLD = 10;
    public static final double CR_POWER = 0.3;
    public static final double SECOND_BEACON_DISTANCE = 30;
    public static final double BEACON_FOUND_THRESHOLD = 4;
    public static final long CR_CENTER_TIME = 1126;
    public static final int SECOND_TURN_ANGLE = 80;
    public static final double FIND_BEACON_POWER = 0.2; // 0.1
    public static final double PUSH_BUTTON_POWER = 0.15; // 0.1
    public static final double CS_BLACK_THRESHOLD = 3;
    public static final double REALIGN_TIME_THRESHOLD = 0.6; // 0.6
    public static final double PUSH_BEACON_TIME = 1.8; // 1.5
    public static final double ARM_POWER = 1.0;
    public static final double ARM_SLOW_POWER = 0.25;
    public static final double ARM_SLEEP_TIME = 0.0; // 0.3 stresses the servo out @ end
    // TODO increase this and test
    public static final double ALIGN_ROTATE_CORRECTION = 6;
    public static final double DRIVE_BACK = 9;

}

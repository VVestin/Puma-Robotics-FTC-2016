package org.firstinspires.ftc.teamcode;

/**
 * Created by sarabranham on 11/19/16.
 */

public enum State {
        PUSH_BEACON_START,		// START state for pushing beacon button. (Does everything from find white line to push button).
        DRIVE_DIST, DRIVE_DIST_LOOP,
        FIND_LINE, FIND_LINE_LOOP, FIND_LINE_SLEEP_STOP,
        ALIGN_LINE, ALIGN_LINE_LOOP, VUFORIA_ALIGN,
        ROTATE_OFF,  ROTATE_OFF_BEGIN, ROTATE_OFF_LOOP, ROTATE_OFF_STOP,
        SLEEP, SLEEP_LOOP,
        DRIVE_TO_BEACON, DRIVE_TO_BEACON_LOOP, DRIVE_TO_BEACON_STOP,
        REALIGN, REALIGN_LOOP,
        SCAN_BEACON, SCAN_BEACON_START, SCAN_BEACON_LOOP,
        SCAN_FOR_BUTTON, SCAN_FOR_BUTTON_LOOP, SCAN_FOR_BUTTON_STOP,
        PUSH_BUTTON, PUSH_BUTTON_STOP,
        DRIVER_CONTROL,
        AUTONOMOUS_START, AUTONOMOUS_STOP, AUTONOMOUS_SECOND_BEACON,
        GO_FOR_TWO,
        HIT_BALL_PAUSE, HIT_BALL_START, HIT_BALL, HIT_BALL_STOP,
        BACK_UP, BACK_UP_LOOP,
        MOTOR_STUCK,
        ROTATE, ROTATE_LOOP,
        FAR_BEACON_ADJ;
}

package org.firstinspires.ftc.teamcode;

/**
 * Created by sarabranham on 11/19/16.
 */

public enum State {
        PUSH_BEACON_START,		// START state for pushing beacon button. (Does everything from find white line to push button).
        DRIVE_DIST, DRIVE_DIST_LOOP,
        FIND_LINE, FIND_LINE_LOOP,
        ALIGN_LINE, ALIGN_LINE_LOOP, VUFORIA_ALIGN,
        SLEEP, SLEEP_LOOP,
        DRIVE_TO_BEACON, DRIVE_TO_BEACON_LOOP, DRIVE_TO_BEACON_STOP,
        SCAN_BEACON, SCAN_BEACON_START, SCAN_BEACON_LOOP,
        SCAN_FOR_BUTTON, SCAN_FOR_BUTTON_LOOP,
        PUSH_BUTTON, PUSH_BUTTON_STOP, DRIVER_CONTROL, AUTONOMOUS_START, AUTONOMOUS_STOP, AUTONOMOUS_SECOND_BEACON,
        CENTER_SERVO, SERVO_STOP;
}

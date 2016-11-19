package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import java.util.Stack;

/**
 * Created by ftcuser1 on 10/1/16.
 */

@TeleOp(name="BasicTeleOp", group="MotorTest")
public class BasicTeleOp extends OpMode implements BeaconConstants {
    private DcMotor left;
    private DcMotor right;

    private OpticalDistanceSensor ods;
    private ColorSensor cs;
    private ColorSensor front;
    private CRServo crservo;
    private boolean stickDrive;
    private boolean beacon;
    private State state;
    private boolean wentLeft = true;
    // Variables state machine uses to pass around parameters:
    private Stack<State> nextStates;
    private double driveDist;
    private double sleepLength;
    private double sleepStart;
    private double initLightVal;
    private double crPower;
    private boolean alignRight;

    public void init() {
        left = hardwareMap.dcMotor.get("left");
        left.setDirection(DcMotor.Direction.REVERSE);
        right = hardwareMap.dcMotor.get("right");
        stickDrive = true;
        beacon = false;
    }

    public void loop() {
        if(!beacon){
            if (stickDrive) {
                left.setPower(-gamepad1.left_stick_y);
                right.setPower(-gamepad1.right_stick_y);
                if (gamepad1.b) {
                    stickDrive = false;
                }
            } else {
                if (gamepad1.right_bumper) {
                    right.setPower(-.5);
                } else {
                    right.setPower(gamepad1.right_trigger);
                }
                if (gamepad1.left_bumper) {
                    left.setPower(-.5);
                } else {
                    left.setPower(gamepad1.left_trigger);
                }
                if (gamepad1.a) {
                    stickDrive = true;
                }
            } if(gamepad1.dpad_left) {
                alignRight = false;
                beacon = true;
                state = State.START;
            } else if(gamepad1.dpad_right){
                alignRight = true;
                beacon = true;
                state = State.START;
            }
            telemetry.addData("driveMode", (stickDrive ? "stick drive" : "trigger drive"));
        } else {
            // State machine for pressing button
            switch (state) {
                case START: //entry point state
                    driveDist = INIT_DRIVE_DISTANCE;
                    state = State.FIND_LINE;
                    nextStates.push(State.ALIGN_LINE);
                    nextStates.push(State.DRIVE_TO_BEACON);
                    nextStates.push(State.PUSH_BUTTON);
                    cs.enableLed(true);
                    break;
                case DRIVE_DIST: //drives forward set d
                    resetEncoders();
                    left.setPower(LINE_FORWARD_POWER);
                    right.setPower(LINE_FORWARD_POWER);
                    state = State.DRIVE_DIST_LOOP;
                    break;
                case DRIVE_DIST_LOOP: //drives forward set d
                    if (left.getCurrentPosition() > AutoDriveOp.TICKS_PER_INCH * driveDist && left.getCurrentPosition() > AutoDriveOp.TICKS_PER_INCH * driveDist) {
                        left.setPower(0);
                        right.setPower(0);
                        state = nextStates.pop();
                    }
                    break;
                case FIND_LINE: //drives forward until line
                    left.setPower(LINE_SLOW_POWER);
                    right.setPower(LINE_SLOW_POWER);
                    state = State.FIND_LINE_LOOP;
                    break;
                case FIND_LINE_LOOP: //stops driving once line
                    if (ods.getLightDetected() - initLightVal > ODS_WHITE_THRESHOLD){
                        left.setPower(0);
                        right.setPower(0);
                        sleepLength = 100;
                        state = State.SLEEP;
                    }
                    break;
                case ALIGN_LINE: //starts turn until on front LS on white line
                    if (alignRight) {
                        right.setPower(-ALIGN_POWER);
                        left.setPower(ALIGN_POWER);
                    } else {
                        right.setPower(ALIGN_POWER);
                        left.setPower(-ALIGN_POWER);
                    }
                    state = State.ALIGN_LINE_LOOP;
                    break;
                case ALIGN_LINE_LOOP: //completes turn - changes states
                    if (avg(front) >= FRONT_WHITE_THRESHOLD) {
                        left.setPower(0);
                        right.setPower(0);
                        sleepLength = 300;
                        state = State.SLEEP;
                    }
                    break;
                case SLEEP:
                    sleepStart = time;
                    state = State.SLEEP_LOOP;
                    break;
                case SLEEP_LOOP:
                    if (time - sleepStart >= sleepLength)
                        state = nextStates.pop();
                    break;
                case DRIVE_TO_BEACON: //drives forward
                    left.setPower(FIND_BEACON_POWER);
                    right.setPower(FIND_BEACON_POWER);
                    state = State.DRIVE_TO_BEACON_LOOP;
                    break;
                case DRIVE_TO_BEACON_LOOP: //completes drive forward
                    if(avg(cs) >= BEACON_FOUND_THRESHOLD){
                        sleepLength = 100;
                        nextStates.push(State.DRIVE_TO_BEACON_STOP);
                        state = State.SLEEP;
                    }
                    break;
                case DRIVE_TO_BEACON_STOP: //stops the motors after sleep
                    left.setPower(0); right.setPower(0);
                    state = nextStates.pop();
                    break;
                case SCAN_BEACON:
                    cs.enableLed(false);
                    sleepLength = 500;
                    nextStates.push(State.SCAN_BEACON_START);
                    state = State.SLEEP;
                    break;
                case SCAN_BEACON_START:
                    crservo.setPower(CR_POWER);
                    state = State.SCAN_BEACON_LOOP;
                    break;
                case SCAN_BEACON_LOOP:
                    if(!(Math.abs(cs.red() - cs.blue()) <= 1 && !(cs.red() > 0 && cs.blue() == 0))){
                        if(cs.red() > cs.blue() != RED_TEAM){
                            wentLeft = false;
                            crservo.setPower(-CR_POWER);
                            nextStates.push(State.SCAN_BEACON_LOOP);
                            sleepLength = 400;
                            state = State.SLEEP;
                        } else {
                            if(!RED_TEAM){
                                sleepLength = 200;
                                nextStates.push(State.SCAN_FOR_BUTTON);
                                state = State.SLEEP;
                            } else {
                                state = State.SCAN_FOR_BUTTON;
                            }
                        }
                    }
                    break;
                case SCAN_FOR_BUTTON:
                    crservo.setPower(wentLeft?1:-1*(CR_POWER/2.));
                    state = State.SCAN_FOR_BUTTON_LOOP;
                    break;
                case SCAN_FOR_BUTTON_LOOP:
                    if(avg() <= CS_BLACK_THRESHOLD){
                        crservo.setPower(0);
                        state = nextStates.pop();
                    }
                    break;
                case PUSH_BUTTON:
                    right.setPower(PUSH_BUTTON_POWER);
                    left.setPower(PUSH_BUTTON_POWER);
                    sleepLength = 1000;
                    nextStates.push(State.PUSH_BUTTON_STOP);
                    state = State.SLEEP;
                    break;
                case PUSH_BUTTON_STOP:
                    right.setPower(0);
                    left.setPower(0);
                    break;
            }

        }

    }
    private enum State {
        START,
        DRIVE_DIST, DRIVE_DIST_LOOP,
        FIND_LINE, FIND_LINE_LOOP,
        ALIGN_LINE, ALIGN_LINE_LOOP,
        SLEEP, SLEEP_LOOP,
        DRIVE_TO_BEACON, DRIVE_TO_BEACON_LOOP, DRIVE_TO_BEACON_STOP,
        SCAN_BEACON, SCAN_BEACON_START, SCAN_BEACON_LOOP,
        SCAN_FOR_BUTTON, SCAN_FOR_BUTTON_LOOP,
        PUSH_BUTTON, PUSH_BUTTON_STOP,

    }

    private void resetEncoders() {
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (right.isBusy() || left.isBusy());
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double avg() {
        return avg(cs);
    }

    public double avg(ColorSensor c) {
        return (c.red() + c.green() + c.blue()) / 3.0;
    }


}

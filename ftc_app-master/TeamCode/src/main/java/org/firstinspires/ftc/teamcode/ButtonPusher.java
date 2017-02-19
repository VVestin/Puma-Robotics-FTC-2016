package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Stack;

public class ButtonPusher extends DriveOp implements BeaconConstants {
    protected OpticalDistanceSensor ods;
    protected ColorSensor cs;
    protected OpticalDistanceSensor front;
    protected CRServo crservo;
    protected Servo shooter;
    protected State state;
    protected boolean wentLeft = true;
    // Variables state machine uses to pass around parameters:
    protected Stack<State> nextStates;
    protected double driveDist;
    protected double sleepLength;
    private double sleepStart;
    protected double rotateAngle;
    protected boolean alignRight;
    private boolean crossedLine;
    private double startRealignTime;
    protected double lastCheckTime;
    protected double lastCheckTicks;
    protected boolean centerServo;
    protected Servo forkDrop;
    protected boolean centerServoMoving;
    private double centerServoStartTime;
    protected boolean recenterServo;
    protected boolean recenterServoMoving;
    private double recenterServoStartTime;
    protected DcMotor arm1;
    protected DcMotor arm2;
    protected int armPos;
    protected double maxLightReading;
    protected double fixStartTime;

    public void init() {
        telemetry.addData("Initializing ButtonPusher", true);
        super.init();
        nextStates = new Stack<State>();
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        cs = hardwareMap.colorSensor.get("color");
        front = hardwareMap.opticalDistanceSensor.get("front");
        crservo = hardwareMap.crservo.get("servo");
        forkDrop = hardwareMap.servo.get("forkDrop");
        shooter = hardwareMap.servo.get("shooter");
        arm1 = hardwareMap.dcMotor.get("arm1");
        arm2 = hardwareMap.dcMotor.get("arm2");
        shooter.setPosition(1);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        telemetry.addData("State", state);
        telemetry.addData("Direction: ", getDirection());
        telemetry.addData("Front Light: ", maxLightReading);
        if (centerServo) {
            centerServoStartTime = time;
            crservo.setPower(CR_POWER);
            centerServo = false;
            centerServoMoving = true;
            telemetry.addData("centeringServo", true);
        }

        if (time - centerServoStartTime > (RED_TEAM?START_SERVO_TIME-.75:START_SERVO_TIME) && centerServoMoving) {
            crservo.setPower(0);
            centerServoMoving = false;
        }

        if (recenterServo) {
            recenterServoStartTime = time;
            crservo.setPower(wentLeft ? -CR_POWER : CR_POWER);
            recenterServo = false;
            recenterServoMoving = true;
            telemetry.addData("recenteringServo", true);
        }
        if (time - recenterServoStartTime > CR_CENTER_TIME && recenterServoMoving) {
            crservo.setPower(0);
            recenterServoMoving = false;
        }
//        if (arm1.getPower() != 1) {
//            if (arm1.getCurrentPosition() < armPos) {
//                arm1.setPower(ARM_SLOW_POWER);
//                arm2.setPower(ARM_SLOW_POWER);
//            } else {
//                arm1.setPower(0);
//                arm2.setPower(0);
//            }
//        }

        switch (state) {
            case PUSH_BEACON_START: // Entry point state
                state = State.FIND_LINE;
                nextStates.push(State.BACK_UP);
                nextStates.push(State.PUSH_BUTTON);
                nextStates.push(State.SCAN_BEACON);
                nextStates.push(State.DRIVE_TO_BEACON);
                nextStates.push(State.ALIGN_LINE);
                armPos = arm1.getCurrentPosition();
                arm1.setPower(0);
                arm2.setPower(0);
                break;
            case DRIVE_DIST: // Drives forward set d
                lastCheckTime = time;
                lastCheckTicks = 0;
                resetEncoders();
                if (driveDist < 0){
                    left.setPower(-LINE_FORWARD_POWER);
                    right.setPower(-LINE_FORWARD_POWER);
                } else {
                    left.setPower(LINE_FORWARD_POWER);
                    right.setPower(LINE_FORWARD_POWER);
                }
                state = State.DRIVE_DIST_LOOP;
                break;
            case DRIVE_DIST_LOOP: // Drives forward set d
                if (time - lastCheckTime > MOTOR_STUCK_THRESHOLD){
                    if (Math.min(Math.abs(left.getCurrentPosition()), Math.abs(right.getCurrentPosition())) - Math.abs(lastCheckTicks) < MOTOR_STUCK_THRESHOLD * TICKS_PER_INCH){
                        left.setPower(0);
                        right.setPower(0);
                        state = State.MOTOR_STUCK;
                    }
                    lastCheckTicks = Math.min(Math.abs(left.getCurrentPosition()), Math.abs(right.getCurrentPosition()));
                    lastCheckTime = time;
                }
                int currentPos = (left.getCurrentPosition() + right.getCurrentPosition()) / 2;
                double power = (LINE_FORWARD_POWER - LINE_MEDIUM_POWER) * (1 - currentPos / (driveDist * TICKS_PER_INCH)) + LINE_MEDIUM_POWER;
//                double power = LINE_FORWARD_POWER;
                if (driveDist < 0){
                    left.setPower(-power);
                    right.setPower(-power);
                } else {
                    left.setPower(power);
                    right.setPower(power);
                }
                if (Math.abs(left.getCurrentPosition()) > TICKS_PER_INCH * Math.abs(driveDist) && Math.abs(right.getCurrentPosition()) > TICKS_PER_INCH * Math.abs(driveDist)) {
                    if (nextStates.peek() != State.PUSH_BEACON_START){
                        left.setPower(0);
                        right.setPower(0);
                    }
                    state = nextStates.pop();
                }
                break;
            case FIND_LINE: // Drives forward until line
                maxLightReading = 0;
                left.setPower(LINE_MEDIUM_POWER);
                right.setPower(LINE_MEDIUM_POWER);
                state = State.FIND_LINE_LOOP;
                break;
            case FIND_LINE_LOOP: // Stops driving once line
                if (front.getRawLightDetected() > 1.0) {
                    telemetry.addData("Slow:", true);
                    left.setPower(LINE_SLOW_POWER);
                    right.setPower(LINE_SLOW_POWER);
                }
                if (front.getRawLightDetected() > maxLightReading) {
                    maxLightReading = front.getRawLightDetected();
                    telemetry.addData("Light Val: ", maxLightReading);
                }
                if (ods.getRawLightDetected() > ODS_WHITE_THRESHOLD - 0.5) {
//                    if (seesWhite(front)) {
//                        left.setPower(0);
//                        right.setPower(0);
//                        sleepLength = .1;
//                        state = State.SLEEP;
//                    }
//                    else {
                        left.setPower(-0.25);
                        right.setPower(-0.25);
                        sleepLength = (RED_TEAM? 0.7: 0.7);
                        state = State.SLEEP;
                        nextStates.push(State.FIND_LINE_FIX);
                        fixStartTime = time;
//                    }
                }
                break;
            case FIND_LINE_FIX:
                if(ods.getRawLightDetected() > (RED_TEAM? 0.8:0.9)){
                    fixStartTime = 0;
                    right.setPower(-LINE_SLOW_POWER);
                    left.setPower(-LINE_SLOW_POWER);
                }else if (ods.getRawLightDetected() < .3) {
                    right.setPower(LINE_SLOW_POWER);
                    left.setPower(LINE_SLOW_POWER);
                    fixStartTime = 0;
                }else{
                    right.setPower(0);
                    left.setPower(0);
                    if (fixStartTime == 0)
                        fixStartTime = time;
//                    sleepLength = .1;
//                    state = State.SLEEP;
                }
                if (fixStartTime != 0 && time - fixStartTime > (RED_TEAM? 0.3:0.5)) {
                    sleepLength = 0.1;
                    state = State.SLEEP;
                }
                break;
            case ROTATE_OFF:
                left.setPower(-LINE_SLOW_POWER);
                right.setPower(-LINE_SLOW_POWER);
                if (ods.getRawLightDetected() > ODS_WHITE_THRESHOLD - 0.15){
                    nextStates.push(State.ROTATE_OFF_LOOP);
                    sleepLength = 0.2;
                    state = State.SLEEP;
                } else {
                    telemetry.addData("Grey: ", true);
                }
                break;
            case ROTATE_OFF_LOOP: // Rotate until not white
                if (seesGrey(ods)) {
                    left.setPower(0);
                    right.setPower(0);
                    state = nextStates.pop();
                }
                break;
            case ROTATE: // TODO uncomment
                gyro.resetZAxisIntegrator();
                state = State.ROTATE_LOOP;
                break;
            case ROTATE_LOOP:
                telemetry.addData("Angle:", rotateAngle);
                telemetry.addData("Current Angle:", getDirection());
                power = (ROTATE_MAX - ROTATE_MIN) * (1 - getDirection() / rotateAngle) + ROTATE_MIN;
                if (rotateAngle < 0){
                    left.setPower(-power);
                    right.setPower(power);
                } else {
                    left.setPower(power);
                    right.setPower(-power);
                }
                if (Math.abs(getDirection()) > Math.abs(rotateAngle) - 1) {
                    left.setPower(0);
                    right.setPower(0);
                    state = nextStates.pop();
                }
//                if (Math.abs(getDirection()) > Math.abs(rotateAngle) + 2) {
//                    // TODO replace magic numbers with constants
//                    if (rotateAngle < 0) {
//                        left.setPower(.15);
//                        right.setPower(-.15);
//                    } else {
//                        right.setPower(.15);
//                        left.setPower(-.15);
//                    }
//                } else if (Math.abs(getDirection()) > Math.abs(rotateAngle) - 1) {
//                    left.setPower(0);
//                    right.setPower(0);
//                    state = nextStates.pop();
//                }
                break;
            case ALIGN_LINE: // Starts turn until on front LS on white line
                crossedLine = false;
                if (alignRight) {
                    right.setPower(-ALIGN_POWER);
                    left.setPower(ALIGN_POWER);
                } else {
                    right.setPower(ALIGN_POWER);
                    left.setPower(-ALIGN_POWER);
                }
                state = State.ALIGN_LINE_LOOP;
                break;
            case ALIGN_LINE_LOOP: // Completes turn - changes states
                if (seesWhite(front)) {
                    if (alignRight) {
                        right.setPower(-ALIGN_POWER);
                        left.setPower(ALIGN_POWER);
                    } else {
                        right.setPower(ALIGN_POWER);
                        left.setPower(-ALIGN_POWER);
                    }
                    fixStartTime = 0;
                    state = State.ALIGN_FRONT_FIX;
//                    left.setPower(0);
//                    right.setPower(0);
//                    sleepLength = .15;
//                    state = State.SLEEP;
                }
                break;
            case ALIGN_FRONT_FIX:
                if(front.getRawLightDetected() > 1.0){
                    fixStartTime = 0;
                    if(alignRight){
                        right.setPower(-ALIGN_POWER);
                        left.setPower(ALIGN_POWER);
                    }else{
                        right.setPower(ALIGN_POWER);
                        left.setPower(-ALIGN_POWER);
                    }
                }else if (front.getRawLightDetected() < .56) {
                    fixStartTime = 0;
                    if(alignRight){
                        right.setPower(ALIGN_POWER);
                        left.setPower(-ALIGN_POWER);
                    }else{
                        right.setPower(-ALIGN_POWER);
                        left.setPower(ALIGN_POWER);
                    }
                }else{
                    right.setPower(0);
                    left.setPower(0);
                    if (fixStartTime == 0)
                        fixStartTime = time;
//                    sleepLength = .1;
//                    state = State.SLEEP;
                }
                if (fixStartTime != 0 && time - fixStartTime > 0.25) {
                    sleepLength = 0.1;
                    state = State.SLEEP;
                }
                break;
            case SLEEP:
                sleepStart = time;
                state = State.SLEEP_LOOP;
                break;
            case SLEEP_LOOP:
                telemetry.addData("sleepdiff: ", time-sleepStart);
                if (time - sleepStart >= sleepLength)
                    state = nextStates.pop();

                break;
            case DRIVE_TO_BEACON:// Drives forward
                cs.enableLed(false);
                cs.enableLed(true);
                left.setPower(FIND_BEACON_POWER);
                right.setPower(FIND_BEACON_POWER);
                state = State.DRIVE_TO_BEACON_LOOP;
                break;
            case DRIVE_TO_BEACON_LOOP: // Completes drive forward
                if (seesGrey(front)) {
//                    state = State.REALIGN;
                }
                if (avg(cs) >= BEACON_FOUND_THRESHOLD) {
//                    left.setPower(0);
//                    right.setPower(0);
                    sleepLength = 0;
                    nextStates.push(State.DRIVE_TO_BEACON_STOP);
                    state = State.SLEEP;
//                    state = nextStates.pop();
                }
                break;
            case DRIVE_TO_BEACON_STOP: // Stops the motors after sleep
                left.setPower(0);
                right.setPower(0);
                state = nextStates.pop();
                break;
            case REALIGN:
                maxLightReading = 0;
                startRealignTime = time;
                if (!RED_TEAM) { // Previously if(true) so red and blue = same
                    right.setPower(0);
                    left.setPower(ALIGN_POWER);
                } else {
                    left.setPower(0);
                    right.setPower(ALIGN_POWER);
//                    left.setPower(-ALIGN_POWER);
                }
                state = State.REALIGN_LOOP;
                break;
            case REALIGN_LOOP:
                if(front.getRawLightDetected() > ODS_WHITE_THRESHOLD){
                    right.setPower(0);
                    left.setPower(0);
                    state = State.DRIVE_TO_BEACON;

                }
                if(front.getRawLightDetected() > maxLightReading) {
                    maxLightReading = front.getRawLightDetected();
                }
//                else if(time - startRealignTime > REALIGN_TIME_THRESHOLD){
//                    if(true) { //alignRight){
//                        right.setPower(ALIGN_POWER);
//                        left.setPower(0);
////                        left.setPower(-ALIGN_POWER);
//                    } else {
////                        right.setPower(-ALIGN_POWER);
//                        left.setPower(ALIGN_POWER);
//                        right.setPower(0);
//                    }
//                    startRealignTime = time + REALIGN_TIME_THRESHOLD;
//                }
                if (avg(cs) >= BEACON_FOUND_THRESHOLD && !RED_TEAM) {
                    left.setPower(0);
                    right.setPower(0);
                    state = nextStates.pop();
                }
                break;
            case SCAN_BEACON:
                cs.enableLed(false);
                sleepLength = 0.5;
                nextStates.push(State.SCAN_BEACON_START);
                state = State.SLEEP;
                break;
            case SCAN_BEACON_START:
                crservo.setPower(CR_FAST_POWER);
                sleepLength = 0.2;
                nextStates.push(State.SCAN_BEACON_LOOP);
                state = State.SLEEP;
                shooter.setPosition(0);
                break;
            case SCAN_BEACON_LOOP:
                if(!(Math.abs(cs.red() - cs.blue()) <= 1 && !(cs.red() > 0 && cs.blue() == 0))){
                    if(cs.red() > cs.blue() != RED_TEAM && wentLeft){
                        crservo.setPower(-CR_FAST_POWER);
//                        crRecenterTime = 2 * time - crRecenterTime;
                        nextStates.push(State.SCAN_BEACON_LOOP);
                        sleepLength = .6;
                        wentLeft = false;
                        state = State.SLEEP;
                    } else {
                        if(cs.red() < cs.blue()){
                            sleepLength = .2;
                            nextStates.push(State.SCAN_FOR_BUTTON);
                            state = State.SLEEP;
                        } else {
                            state = State.SCAN_FOR_BUTTON;
                        }
                    }
                }
                break;
            case SCAN_FOR_BUTTON:
                cs.enableLed(true);
                crservo.setPower(wentLeft?1:-1*(CR_FAST_POWER/3.));
                sleepLength = .3;
                nextStates.push(State.SCAN_FOR_BUTTON_LOOP);
                state = State.SLEEP;
                break;
            case SCAN_FOR_BUTTON_LOOP:
                if(avg() <= CS_BLACK_THRESHOLD) {
                    crservo.setPower(0);
                    state = nextStates.pop();
                }
                break;
            case PUSH_BUTTON:
                right.setPower(PUSH_BUTTON_POWER);
                left.setPower(PUSH_BUTTON_POWER);
                cs.enableLed(false);
                sleepLength = PUSH_BEACON_TIME;
                nextStates.push(State.PUSH_BUTTON_STOP);
                state = State.SLEEP;
                break;
            case PUSH_BUTTON_STOP:
                crservo.setPower(wentLeft? -CR_FAST_POWER : CR_FAST_POWER);
                right.setPower(0);
                left.setPower(0);
                sleepLength = 0.3;
                state = State.SLEEP;
                break;
            case BACK_UP:
                sleepLength = .5;
                crservo.setPower(0);
                right.setPower(-(LINE_MEDIUM_POWER));
                left.setPower(-(LINE_MEDIUM_POWER));
                recenterServo = true;
                resetEncoders();
                nextStates.push(State.BACK_UP_LOOP);
                state = State.SLEEP;
                gyro.resetZAxisIntegrator();
                break;
            case BACK_UP_LOOP:
                if (RED_TEAM) {
                    left.setPower(0);
                } else {
                    right.setPower(0);
                }
                if (Math.abs(getDirection()) > (RED_TEAM?79:75)) {
                    right.setPower(0);
                    left.setPower(0);
                    state = nextStates.pop();
                }
                break;
            case MOTOR_STUCK:
                telemetry.addData("STUCK!", true);
                break;
            case LEFT_TEST:
                if(front.getRawLightDetected() > 1) {
                    maxLightReading = front.getRawLightDetected();
                    state = State.DRIVER_CONTROL;
                }
                right.setPower(ALIGN_POWER);
                break;
            case RIGHT_TEST:
                if(front.getRawLightDetected() > 1) {
                    maxLightReading = front.getRawLightDetected();
                    state = State.DRIVER_CONTROL;
                }
                left.setPower(ALIGN_POWER);
                break;
        }

    }

    public boolean seesWhite(OpticalDistanceSensor light){
        double diff = light.getRawLightDetected();
        if (diff > ODS_WHITE_THRESHOLD) {
            return true;
        }else{
            return false;
        }
    }

    public boolean seesGrey(OpticalDistanceSensor light){
        double diff = light.getRawLightDetected();
        if (diff < ODS_GREY_THRESHOLD) {
            return true;
        }else{
            return false;
        }
    }

    public double avg() {
        return avg(cs);
    }

    public double avg(ColorSensor c) {
        return (c.red() + c.green() + c.blue()) / 3.0;
    }
}

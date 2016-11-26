package org.firstinspires.ftc.teamcode;

public class ButtonPusher extends DriveOp implements BeaconConstants {
	private OpticalDistanceSensor ods;
    private ColorSensor cs;
    private ColorSensor front;
    private CRServo crservo;
    protected State state;
    protected boolean wentLeft = true;
    // Variables state machine uses to pass around parameters:
    protected Stack<State> nextStates;
    protected double driveDist;
    protected double sleepLength;
    private double sleepStart;
    private double initLightVal;
    private double crPower;
    private boolean alignRight;

	public void init() {
		super.init();
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        cs = hardwareMap.colorSensor.get("color");
        front = hardwareMap.colorSensor.get("front");
        crservo = hardwareMap.crservo.get("servo");
	}

	public void loop() {
		telemetry.addData("State", state);
		switch (state) {
			case PUSH_BEACON_BUTTON: //entry point state
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
				state = nextStates.pop();
                break;
        }

	}
    
    public double avg() {
        return avg(cs);
    }

    public double avg(ColorSensor c) {
        return (c.red() + c.green() + c.blue()) / 3.0;
    }
}

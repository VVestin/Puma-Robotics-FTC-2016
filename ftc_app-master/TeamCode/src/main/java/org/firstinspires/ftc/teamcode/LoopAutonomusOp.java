package org.firstinspires.ftc.teamcode;

public class LoopAutonomosOp extends ButtonPusher {

	public void init() {
		super.init();
		state = State.AUTONOMOUS_START;
	}

	public void loop() {
		switch(state) {
			case State.AUTONOMOUS_START:
				nextStates.push(State.PUSH_BEACON_BUTTON);
				nextStates.push(State.AUTONOMOUS_SECOND_BEACON);
				driveDist = INIT_DRIVE_DIST; 
				state = State.DRIVE_DIST;
				break;
			case State.AUTONOMOUS_SECOND_BEACON:
				nextStates.push(State.AUTONOMOUS_STOP);
				// TODO add 90 degree turn and drive.
				state = State.PUSH_BEACON_BUTTON;
				break;
			case State.AUTONOMOUS_STOP:
				// Do nothing, wait for time to run out.
				break;
			default:
				super.loop();
		}
	}
}

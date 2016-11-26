package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutonomousLoop")

public class LoopAutonomousOp extends ButtonPusher implements BeaconConstants {

	public void init() {
		super.init();
		state = State.AUTONOMOUS_START;
	}

	public void loop() {
		switch(state) {
			case AUTONOMOUS_START:
				nextStates.add(State.PUSH_BEACON_START);
				nextStates.add(State.AUTONOMOUS_SECOND_BEACON);
				driveDist = INIT_DRIVE_DISTANCE;
				state = State.DRIVE_DIST;
				break;
			case AUTONOMOUS_SECOND_BEACON:
				nextStates.add(State.AUTONOMOUS_STOP);
				// TODO add 90 degree turn and drive.
				state = State.PUSH_BEACON_START;
				break;
			case AUTONOMOUS_STOP:
				// Do nothing, wait for time to run out.
				break;
			default:
				super.loop();
		}
	}
}

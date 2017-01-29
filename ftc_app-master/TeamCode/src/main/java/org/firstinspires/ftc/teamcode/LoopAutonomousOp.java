package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="TwoBeacons")

public class LoopAutonomousOp extends ButtonPusher implements BeaconConstants {

	public void init() {
		super.init();
		state = State.AUTONOMOUS_START;
	}

	public void loop() {
		switch(state) {
			case AUTONOMOUS_START:
				nextStates.push(State.AUTONOMOUS_SECOND_BEACON);
				nextStates.push(State.PUSH_BEACON_START);
				driveDist = INIT_DRIVE_DISTANCE;
				alignRight = !RED_TEAM;
				state = State.DRIVE_DIST;
				break;
			case AUTONOMOUS_SECOND_BEACON:
				nextStates.push(State.AUTONOMOUS_STOP);
				if (RED_TEAM) {
					rotateAngle = 84;
				} else {
					rotateAngle = -84;
				}
				state = State.ROTATE;
				driveDist = 30;
				nextStates.push(State.PUSH_BEACON_START);

				// TODO add 90 degree turn and drive.
				nextStates.push(State.DRIVE_DIST);
				break;
			case AUTONOMOUS_STOP:
				// Do nothing, wait for time to run out.
				break;
			default:
				super.loop();
		}
	}
}

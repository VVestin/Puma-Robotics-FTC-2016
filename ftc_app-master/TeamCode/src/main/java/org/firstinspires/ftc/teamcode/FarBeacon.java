package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


public class FarBeacon extends ButtonPusher implements BeaconConstants {

    public void init() {
        super.init();
        state = State.AUTONOMOUS_START;
    }

    public void loop() {
        telemetry.addData("State:", state);
        switch (state) {
            case AUTONOMOUS_START:
                nextStates.push(State.HIT_BALL_STOP);
                nextStates.push(State.HIT_BALL_START);
                nextStates.push(State.PUSH_BEACON_START);
                nextStates.push(State.FAR_BEACON_ADJ);
                driveDist = 70;
                alignRight = !RED_TEAM;
                state = State.DRIVE_DIST;
                break;
            case FAR_BEACON_ADJ:
                driveDist = 24;
                rotateAngle = RED_TEAM?-20:20;
                state = State.ROTATE;
                nextStates.push(State.DRIVE_DIST);
                break;
            case HIT_BALL_START:
                rotateAngle = RED_TEAM?45:-45;
                driveDist = -70;
                state = State.ROTATE;
                nextStates.push(State.DRIVE_DIST);
                break;
            case HIT_BALL_STOP:
                break;
            default:
                super.loop();
                break;
        }
    }
}
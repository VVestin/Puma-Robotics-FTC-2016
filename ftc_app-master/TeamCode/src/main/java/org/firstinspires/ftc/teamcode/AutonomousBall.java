package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutonomousBall")

public class AutonomousBall extends ButtonPusher implements BeaconConstants {

    public void init() {
        super.init();
        state = State.AUTONOMOUS_START;
    }

    public void loop() {
        telemetry.addData("State:", state);
        switch (state) {
            case AUTONOMOUS_START:
                nextStates.push(State.HIT_BALL_STOP);
                nextStates.push(State.HIT_BALL);
                nextStates.push(State.HIT_BALL_START);
                nextStates.push(State.PUSH_BEACON_START);
                driveDist = INIT_DRIVE_DISTANCE;
                alignRight = !RED_TEAM;
                state = State.DRIVE_DIST;
                break;
            case HIT_BALL_START:
                rotateAngle = RED_TEAM?-40:40;
                driveDist = -36;
                state = State.ROTATE;
                nextStates.push(State.DRIVE_DIST);
//                driveDist = -60;
//                rotateAngle = RED_TEAM?45:-45;
//                state = State.DRIVE_DIST;
//                nextStates.push(State.ROTATE);
                break;
            case HIT_BALL:
                rotateAngle = RED_TEAM?80:-80;
                driveDist = -24;
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
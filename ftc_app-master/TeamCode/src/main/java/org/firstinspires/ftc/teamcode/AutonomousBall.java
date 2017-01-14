package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutonomousBall")

public class AutonomousBall extends ButtonPusher implements BeaconConstants {

    public void init() {
        super.init();
        state = State.AUTONOMOUS_START;
    }

    public void start() {
        forkDrop.setPosition(BasicTeleOp.FORK_LOCKED);
    }

    public void loop() {
        telemetry.addData("State:", state);
        switch (state) {
            case AUTONOMOUS_START:
                if (BALL_ONLY == true){
                    driveDist = 54;
                    nextStates.push(State.HIT_BALL_STOP);
                    state = State.DRIVE_DIST;
                } else {
                    nextStates.push(State.HIT_BALL_STOP);
                    if (!SECOND_BEACON && !RAMP) {
                        nextStates.push(State.HIT_BALL);
                    }
                    nextStates.push(State.HIT_BALL_START);
                    nextStates.push(State.PUSH_BEACON_START);
                    nextStates.push(State.DRIVE_DIST);
                    centerServo = true;
                    if (SECOND_BEACON && !FOURTH_SQUARE) {
                        driveDist = 83;
                    } else {
                        driveDist = INIT_DRIVE_DISTANCE;
                    }
                    alignRight = !RED_TEAM;
                    arm1.setPower(ARM_POWER);
                    arm2.setPower(ARM_POWER);
                    sleepLength = ARM_SLEEP_TIME;
                    state = State.SLEEP;
                }
                break;
            case HIT_BALL_START:
                if (RAMP) {
                    rotateAngle = RED_TEAM?-110:100;
                    driveDist = 36;
                } else if (SECOND_BEACON) {
                    rotateAngle = RED_TEAM?26:-28;
                    driveDist = RED_TEAM?-65:-60;
                } else {
                    rotateAngle = RED_TEAM?-35:35;
                    driveDist = -36;

                }
                state = State.ROTATE;
                nextStates.push(State.DRIVE_DIST);
//                driveDist = -60;
//                rotateAngle = RED_TEAM?45:-45;
//                state = State.DRIVE_DIST;
//                nextStates.push(State.ROTATE);
                break;
            case HIT_BALL:
                rotateAngle = RED_TEAM?75:-85;
                driveDist = -36;
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
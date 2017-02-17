package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous")
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
        telemetry.addData("Color:", avg(cs));
        switch (state) {
            case AUTONOMOUS_START:
                nextStates.push(State.HIT_BALL_STOP);
                nextStates.push(State.GO_FOR_TWO);
                nextStates.push(State.PUSH_BEACON_START);
                nextStates.push(State.DRIVE_DIST);
                centerServo = true;
                driveDist = INIT_DRIVE_DISTANCE;
                alignRight = !RED_TEAM;
                sleepLength = ARM_SLEEP_TIME;
                state = State.SLEEP;
                break;
            case GO_FOR_TWO:
                nextStates.push(State.PUSH_BEACON_START);
                driveDist = RAMP ? -30 : 20;
                state = State.DRIVE_DIST;
                right.setPower(0);
                left.setPower(0);
            default:
                super.loop();
                break;
        }
    }
}
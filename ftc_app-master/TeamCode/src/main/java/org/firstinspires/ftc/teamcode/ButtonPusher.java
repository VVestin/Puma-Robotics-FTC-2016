package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Stack;

import static android.graphics.PathDashPathEffect.Style.ROTATE;
import static java.lang.Integer.parseInt;
import static java.lang.Thread.sleep;

@TeleOp(name="MrDButtonPusher")
public class ButtonPusher extends DriveOp implements BeaconConstants {
    protected OpticalDistanceSensor ods;
    protected ColorSensor cs;
    protected OpticalDistanceSensor front;
    protected CRServo crservo;
    protected State state;
    protected boolean wentLeft = true;
    // Variables state machine uses to pass around parameters:
    protected Stack<State> nextStates;
    protected double driveDist;
    protected double sleepLength;
    private double sleepStart;
    protected double rotateAngle;
    private double initLightVal;
    private double crPower;
    protected boolean alignRight;
    private boolean crossedLine;
    private double startRealignTime;
    protected double lastCheckTime;
    protected double lastCheckTicks;
    protected boolean centerServo;
    protected Servo forkDrop;
    protected boolean centerServoMoving;
    private double centerServoStartTime;

    //Vuforia stuff
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable wheels;
    private VuforiaTrackable tools;
    private VuforiaTrackable legos;
    private VuforiaTrackable gears;
    private VuforiaTrackableDefaultListener wheelListener;
    private VuforiaTrackableDefaultListener toolListener;
    private VuforiaTrackableDefaultListener legoListener;
    private VuforiaTrackableDefaultListener gearListener;

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    public static final String VUFORIA_KEY = "AV4ONxv/////AAAAGefaDmLKjkgWifNHOt4h8QgFb23EhhiUz7Po/rcnXDMuJHa2Okvh/NLUSza5phLaIuyvWUINyu/cyKpmChyUCJ/A05QHnq04DK6FE36G2ihZTKbHfaJc/sz3LBIGnNa0Hwv+NZCYxNKsnm5IDBDx//c6btS/v1+6ESNE2YdieabitaPyH0RDBppIRcX2ufK6Fk71GydEz2pXkfnG8QN1zJRJn+PHf1Gg70SLF/aXHhGBVyudSlMk+EE89Or5ZyJLCSmUbS0LAHoBiVoSUtFb25iMSd/Zf3DsBPr/hZGKTfd7/c6BqeSKOidNPnOryVYThQM3hec5sbToDLneUqyhXRlAiifCw7x0he3XfFJp+NH0"; // Insert your own key here

    public void init() {
        telemetry.addData("Initializing ButtonPusher", true);
        super.init();
        nextStates = new Stack<State>();
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        cs = hardwareMap.colorSensor.get("color");
        front = hardwareMap.opticalDistanceSensor.get("front");
        crservo = hardwareMap.crservo.get("servo");
        forkDrop = hardwareMap.servo.get("forkDrop");
        lastKnownLocation=createMatrix(0, 0, 0, 0, 0, 0);
//        setupVuforia();
    }

    public void loop() {
        telemetry.addData("State", state);
        if (centerServo) {
            centerServoStartTime = time;
            crservo.setPower(CR_POWER);
            centerServo = false;
            centerServoMoving = true;
        }
        if (time - centerServoStartTime > START_SERVO_TIME && centerServoMoving) {
            crservo.setPower(0);
            centerServoMoving = false;
        }
        switch (state) {
            case PUSH_BEACON_START: // Entry point state
                state = State.FIND_LINE;
                nextStates.push(State.BACK_UP);
                nextStates.push(State.PUSH_BUTTON);
                nextStates.push(State.SCAN_BEACON);
                //nextStates.push(State.VUFORIA_ALIGN);
                nextStates.push(State.DRIVE_TO_BEACON);
                nextStates.push(State.ALIGN_LINE);
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
                double leftPower = (LINE_FORWARD_POWER - LINE_SLOW_POWER) * (1 - left.getCurrentPosition() / (driveDist * TICKS_PER_INCH)) + LINE_SLOW_POWER;
                double rightPower = (LINE_FORWARD_POWER - LINE_SLOW_POWER) * (1 - right.getCurrentPosition() / (driveDist * TICKS_PER_INCH)) + LINE_SLOW_POWER;
                if (driveDist < 0){
                    left.setPower(-leftPower);
                    right.setPower(-rightPower);
                } else {
                    left.setPower(leftPower);
                    right.setPower(rightPower);
                }
                if (Math.abs(left.getCurrentPosition()) > AutoDriveOp.TICKS_PER_INCH * Math.abs(driveDist) && Math.abs(right.getCurrentPosition()) > AutoDriveOp.TICKS_PER_INCH * Math.abs(driveDist)) {
                    if (nextStates.peek() != State.PUSH_BEACON_START){
                        left.setPower(0);
                        right.setPower(0);
                    }
                    state = nextStates.pop();
                }
                break;
            case FIND_LINE: // Drives forward until line
                left.setPower(LINE_SLOW_POWER);
                right.setPower(LINE_SLOW_POWER);
                state = State.FIND_LINE_LOOP;
                break;
            case FIND_LINE_LOOP: // Stops driving once line
                if (seesWhite(ods)){
                    if (seesWhite(front)) {
                        left.setPower(0);
                        right.setPower(0);
                        sleepLength = .1;
                        state = State.SLEEP;
                    } else {;
                        left.setPower(0);
                        right.setPower(0);
                        state = State.ROTATE_OFF;
                    }
                }
                break;
            case ROTATE_OFF:
                left.setPower(-LINE_SLOW_POWER);
                right.setPower(-LINE_SLOW_POWER);
                if (seesWhite(ods)){
                    nextStates.push(State.ROTATE_OFF_LOOP);
                    sleepLength = 0.2;
                    state = State.SLEEP;
                } else {
                    telemetry.addData("Grey: ", true);
                }
                break;
            case ROTATE_OFF_LOOP: // Rotate until not white
                if (!seesWhite(ods)) {
                    left.setPower(0);
                    right.setPower(0);
                    state = nextStates.pop();
                }
                break;
            case ROTATE: // TODO uncomment
                gyro.resetZAxisIntegrator();
                if (rotateAngle < 0) {
                    left.setPower(-.25);
                    right.setPower(.25);
                } else {
                    right.setPower(-.25);
                    left.setPower(.25);
                }
                state = State.ROTATE_LOOP;
                break;
            case ROTATE_LOOP:
                if (Math.abs(getDirection()) > Math.abs(rotateAngle) + 2) {
                    // TODO replace magic numbers with constants
                    if (rotateAngle < 0) {
                        left.setPower(.15);
                        right.setPower(-.15);
                    } else {
                        right.setPower(.15);
                        left.setPower(-.15);
                    }
                } else if (Math.abs(getDirection()) > Math.abs(rotateAngle) - 1) {
                    left.setPower(0);
                    right.setPower(0);
                    state = nextStates.pop();
                }
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
                    left.setPower(0);
                    right.setPower(0);
                    sleepLength = .15;
                    state = State.SLEEP;
                }
                break;
            case VUFORIA_ALIGN:
                visionTargets.activate();
                OpenGLMatrix latestLocation = getLocation();

                if (latestLocation != null) {
                    lastKnownLocation = latestLocation;
                }

                int angle = getAngleFromMatrix(lastKnownLocation);
                int anglebuffer = 2; // Tweak this

                if (Math.abs(angle) - 90 > anglebuffer) {
                    if ((angle < 0 && angle > -90 + anglebuffer) || (angle > 0 && angle > 90 + anglebuffer)) {
                        left.setPower(ALIGN_POWER);
                        right.setPower(-ALIGN_POWER);
                    } else if ((angle < 0 && angle < -90 - anglebuffer) || (angle > 0 && angle < 90 - anglebuffer)) {
                        left.setPower(-ALIGN_POWER);
                        right.setPower(ALIGN_POWER);
                    }

                    latestLocation = getLocation();
                    if (latestLocation != null) {
                        lastKnownLocation = latestLocation;
                    }
                    angle = getAngleFromMatrix(lastKnownLocation);

//                    telemetry.addData("angle", angle);
//                    telemetry.update();
                }

                visionTargets.deactivate();
                break;
            case SLEEP:
                sleepStart = time;
                state = State.SLEEP_LOOP;
                break;
            case SLEEP_LOOP:
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
                if (!seesWhite(front)) {
                    state = State.REALIGN;
                }
                if (avg(cs) >= BEACON_FOUND_THRESHOLD) {
                    left.setPower(0);
                    right.setPower(0);
                    //sleepLength = .1;
                    //nextStates.push(State.DRIVE_TO_BEACON_STOP);
                    //state = State.SLEEP;
                    state = nextStates.pop();
                }
                break;
            case DRIVE_TO_BEACON_STOP: // Stops the motors after sleep
                left.setPower(0);
                right.setPower(0);
                state = nextStates.pop();
                break;
            case REALIGN:
                startRealignTime = time;
                if (true) { //alignRight) {
                    right.setPower(-ALIGN_POWER);
                    left.setPower(ALIGN_POWER);
                } else {
                    right.setPower(ALIGN_POWER);
                    left.setPower(-ALIGN_POWER);
                }
                state = State.REALIGN_LOOP;
                break;
            case REALIGN_LOOP:
                if(seesWhite(front)){
                    right.setPower(0);
                    left.setPower(0);
                    state = State.DRIVE_TO_BEACON;
                } else if(time - startRealignTime > REALIGN_TIME_THRESHOLD){
                    if(true) { //alignRight){
                        right.setPower(ALIGN_POWER);
                        left.setPower(-ALIGN_POWER);
                    } else {
                        right.setPower(-ALIGN_POWER);
                        left.setPower(ALIGN_POWER);
                    }
                    startRealignTime = time + REALIGN_TIME_THRESHOLD;
                }
                if (avg(cs) >= BEACON_FOUND_THRESHOLD && !RED_TEAM) {
                    left.setPower(0);
                    right.setPower(0);
                    state = nextStates.pop();
                }
                break;
            case SCAN_BEACON:
                cs.enableLed(false);
                sleepLength = .5;
                nextStates.push(State.SCAN_BEACON_START);
                state = State.SLEEP;
                break;
            case SCAN_BEACON_START:
                crservo.setPower(CR_POWER);
                state = State.SCAN_BEACON_LOOP;
                break;
            case SCAN_BEACON_LOOP:
                if(!(Math.abs(cs.red() - cs.blue()) <= 1 && !(cs.red() > 0 && cs.blue() == 0))){
                    if(cs.red() > cs.blue() != RED_TEAM && wentLeft){
                        crservo.setPower(-CR_POWER);
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
                crservo.setPower(wentLeft?1:-1*(CR_POWER/3.));
                sleepLength = .3;
                nextStates.push(State.SCAN_FOR_BUTTON_LOOP);
                state = State.SLEEP;
                break;
            case SCAN_FOR_BUTTON_LOOP:
                if(avg() <= CS_BLACK_THRESHOLD){
                    if (true){ //!RED_TEAM) {
                        sleepLength = 0.1;
                        nextStates.push(State.SCAN_FOR_BUTTON_STOP);
                        state = State.SLEEP;
                    } else {
                        state = State.SCAN_FOR_BUTTON_STOP;
                    }
                }
                break;
            case SCAN_FOR_BUTTON_STOP:
                crservo.setPower(0);
                state = nextStates.pop();
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
                right.setPower(0);
                left.setPower(0);
                state = nextStates.pop();
                break;
            case BACK_UP:
                crservo.setPower(wentLeft ? -CR_POWER : CR_POWER);
                sleepLength = CR_CENTER_TIME/1000d;
                right.setPower(-LINE_SLOW_POWER);
                left.setPower(-LINE_SLOW_POWER);
                resetEncoders();
                nextStates.push(State.BACK_UP_LOOP);
                state = State.SLEEP;
                break;
            case BACK_UP_LOOP:
                // TODO Back up and turn 90 degrees to save time
                crservo.setPower(0);
                right.setPower(-LINE_SLOW_POWER);
                left.setPower(-LINE_SLOW_POWER);
                if (Math.max(left.getCurrentPosition(), right.getCurrentPosition()) < -DRIVE_BACK * TICKS_PER_INCH){
                    right.setPower(0);
                    left.setPower(0);
                    state = nextStates.pop();
                }
                break;
            case MOTOR_STUCK:
                telemetry.addData("STUCK!", true);
                break;
        }

    }

    public boolean seesWhite(OpticalDistanceSensor light){
        double diff = light.getRawLightDetected() - initLightVal;
        if(diff > ODS_WHITE_THRESHOLD){
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

    public void setupVuforia() {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");

        // Setup the targets to be tracked
        //origin for coordinate system is set to the red corner.
        //+x direction is set to side without beacons,
        //+y direction set to side with beacons,
        //+z direction set to out of field.
        wheels = visionTargets.get(0); // 0 corresponds to the wheels target
        wheels.setName("Wheels");
        wheels.setLocation(createMatrix(2100, 3600, 150, 90, 0, 0));

        tools = visionTargets.get(1);
        tools.setName("Tools");
        tools.setLocation(createMatrix(0, 2700, 150, 90, 0, 90));

        legos = visionTargets.get(2);
        legos.setName("Legos");
        legos.setLocation(createMatrix(900, 3600, 150, 90, 0, 0));

        gears = visionTargets.get(3);
        gears.setName("Gears of War");//:P
        gears.setLocation(createMatrix(0, 1500, 150, 90, 0, 90));

        // Set phone location on robot
        phoneLocation = createMatrix(0, 0, 0, 90, 0, 0);

        // Setup listeners and informs them of phone information
        wheelListener = (VuforiaTrackableDefaultListener) wheels.getListener();
        wheelListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        toolListener = (VuforiaTrackableDefaultListener) tools.getListener();
        toolListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        legoListener = (VuforiaTrackableDefaultListener) legos.getListener();
        legoListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        gearListener = (VuforiaTrackableDefaultListener) gears.getListener();
        gearListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    public OpenGLMatrix getLocation(){
        OpenGLMatrix location = createMatrix(0, 0, 0, 0, 0, 0);//just set to orign since it'll get updated no matter what at this location on the field

        if(gearListener.isVisible()){ //if gears picture is visible set location based on that picture
            location = gearListener.getUpdatedRobotLocation();
        }else if(toolListener.isVisible()){ //if tools picture is visible set location based on that picture
            location = toolListener.getUpdatedRobotLocation();
        }else if(wheelListener.isVisible()){ //if wheels picture is visible set location based on that picture
            location = wheelListener.getUpdatedRobotLocation();
        }else if(legoListener.isVisible()) { //if legos picture is visible set location based on that picture
            location = legoListener.getUpdatedRobotLocation();
        }

        return location;

    }

    public int getAngleFromMatrix(OpenGLMatrix matrix){
        String m=formatMatrix(matrix);
        int start=m.indexOf('Z');
        int end=m.indexOf(' ', start+2);
        return parseInt(m.substring(start+2, end));
    }
    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    public String formatMatrix(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

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

import java.util.Stack;

import static java.lang.Integer.parseInt;

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
    protected boolean alignRight;

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
		super.init();
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        cs = hardwareMap.colorSensor.get("color");
        front = hardwareMap.colorSensor.get("front");
        crservo = hardwareMap.crservo.get("servo");
        lastKnownLocation=createMatrix(0, 0, 0, 0, 0, 0);
        setupVuforia();
	}

	public void loop() {
		telemetry.addData("State", state);
		switch (state) {
			case PUSH_BEACON_BUTTON: // Entry point state
				driveDist = INIT_DRIVE_DISTANCE;
                state = State.FIND_LINE;
                nextStates.push(State.ALIGN_LINE);
                nextStates.push(State.VUFORIA_ALIGN);
                nextStates.push(State.DRIVE_TO_BEACON);
                nextStates.push(State.PUSH_BUTTON);
                cs.enableLed(true);
                break;
            case DRIVE_DIST: // Drives forward set d
                resetEncoders();
                left.setPower(LINE_FORWARD_POWER);
                right.setPower(LINE_FORWARD_POWER);
                state = State.DRIVE_DIST_LOOP;
                break;
            case DRIVE_DIST_LOOP: // Drives forward set d
                if (left.getCurrentPosition() > AutoDriveOp.TICKS_PER_INCH * driveDist && left.getCurrentPosition() > AutoDriveOp.TICKS_PER_INCH * driveDist) {
                    left.setPower(0);
                    right.setPower(0);
                    state = nextStates.pop();
                }
                break;
            case FIND_LINE: // Drives forward until line
                left.setPower(LINE_SLOW_POWER);
                right.setPower(LINE_SLOW_POWER);
                state = State.FIND_LINE_LOOP;
                break;
            case FIND_LINE_LOOP: // Stops driving once line
                if (ods.getLightDetected() - initLightVal > ODS_WHITE_THRESHOLD){
                    left.setPower(0);
                    right.setPower(0);
                    sleepLength = 100;
                    state = State.SLEEP;
                }
                break;
            case ALIGN_LINE: // Starts turn until on front LS on white line
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
                if (avg(front) >= FRONT_WHITE_THRESHOLD) {
                    left.setPower(0);
                    right.setPower(0);
                    sleepLength = 300;
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
                int anglebuffer = 2; //tweak this

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
            case DRIVE_TO_BEACON: // Drives forward
                left.setPower(FIND_BEACON_POWER);
                right.setPower(FIND_BEACON_POWER);
                state = State.DRIVE_TO_BEACON_LOOP;
                break;
            case DRIVE_TO_BEACON_LOOP: // Completes drive forward
                if(avg(cs) >= BEACON_FOUND_THRESHOLD){
                    sleepLength = 100;
                    nextStates.push(State.DRIVE_TO_BEACON_STOP);
                    state = State.SLEEP;
                }
                break;
            case DRIVE_TO_BEACON_STOP: // Stops the motors after sleep
                left.setPower(0); right.setPower(0);
                state = nextStates.pop();
                break;

            // Ask for user input - if RT power = - (right), else LT power = + (left)

            case SCAN_BEACON: // Turns LED off
                cs.enableLed(false);
                sleepLength = 500;
                nextStates.push(State.SCAN_BEACON_START);
                state = State.SLEEP;
                break;
            case SCAN_BEACON_START: // Starts servo moving (left is +)
                crservo.setPower(CR_POWER);
                state = State.SCAN_BEACON_LOOP;
                break;
            case SCAN_BEACON_LOOP: // Goes forward or turns
                if(!(Math.abs(cs.red() - cs.blue()) <= 1 && !(cs.red() > 0 && cs.blue() == 0))){
                    if(cs.red() > cs.blue() != RED_TEAM && wentLeft){ // If wrong color - turn
                        wentLeft = false;
                        crservo.setPower(-CR_POWER);
                        nextStates.push(State.SCAN_BEACON_LOOP);
                        sleepLength = 400;
                        state = State.SLEEP;
                    } else {
                        if(cs.red() < cs.blue()){ // Small fix
                            sleepLength = 200;
                            nextStates.push(State.SCAN_FOR_BUTTON);
                            state = State.SLEEP;
                        } else {
                            state = State.SCAN_FOR_BUTTON;
                        }
                    }
                }
                break;
            case SCAN_FOR_BUTTON: // Keeps going in same direction - slows down
                cs.enableLed(true);
                crservo.setPower(wentLeft?1:-1*(CR_POWER/2.));
                state = State.SCAN_FOR_BUTTON_LOOP;
                break;
            case SCAN_FOR_BUTTON_LOOP: // Stops when it sees black
                if(avg() <= CS_BLACK_THRESHOLD){
                    crservo.setPower(0);
                    state = nextStates.pop();
                }
                break;
            case PUSH_BUTTON: // Drives forward
                right.setPower(PUSH_BUTTON_POWER);
                left.setPower(PUSH_BUTTON_POWER);
                sleepLength = 1000;
                nextStates.push(State.PUSH_BUTTON_STOP);
                state = State.SLEEP;
                break;
            case PUSH_BUTTON_STOP: // Finishes drive forward
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

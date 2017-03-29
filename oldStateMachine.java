package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.HardwareClass;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ga on 3/7/16.
 */

enum State {INITIALIZE,
    LAUNCH_BALLS,
    MOVE_TO_BEACON,
    TURN_TO_BEACON,
    DRIVE_TO_LINE,
    LINE_FOLLOW,
    SQUARE_UP,
    DETECT_COLOR,
    PUSH_BUTTON,
    DRIVE_TO_PLATFORM,
    PARK_ON_PLATFORM,
    STOP
}
// possible values for state because it is an enum type

@Autonomous(name="working State Machine RED")
@Disabled

public class oldStateMachine extends HardwareClass {

    State state;    // declares variable state of type State

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state



    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;

    public double robotX;
    public double robotY;
    public double robotBearing;
    public double speed;
    public List<VuforiaTrackable> allTrackables;

    public double perfectColorValue = .05;
    public double sensorInput;
    public double correction;
    private double GRAY_VALUE = 0.02; // gray values never go above .02, so this is the threshold
    // white value is around .08

    public boolean theJudgment; //true means correcting from too white, false means correcting from too gray
    @Override
    public void init() {

        lineSensor = hardwareMap.opticalDistanceSensor.get("lsense");
        motorRightFront = hardwareMap.dcMotor.get("rf");
        motorRightRear = hardwareMap.dcMotor.get("rb");

        motorLeftFront = hardwareMap.dcMotor.get("lf");
        motorLeftRear = hardwareMap.dcMotor.get("lb");

        try{
        ballFlipper = hardwareMap.dcMotor.get("flip");

        //set directions of motors when driving
        motorLeftRear.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightRear.setDirection(DcMotor.Direction.REVERSE);

        // assigns state variable to enum INITIALIZE
        state = State.INITIALIZE;

        // reset encoder target positions to 0 on drive wheels and ball flipper
        resetDriveEncoders();
        ballFlipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        catch (IllegalArgumentException i){}
        catch (NullPointerException n){}

        // set drive power to 0
        motorRightFront.setPower(0);
        motorLeftFront.setPower(0);
        motorRightRear.setPower(0);
        motorLeftRear.setPower(0);

        telemetry.addData("1", String.format("Left Front: %5d  Right Front: %5d ",
                motorRightFront.getCurrentPosition(),
                motorLeftFront.getCurrentPosition()));

        VuforiaLocalizer vuforia;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AY7wJZH/////AAAAGZMdB1/VKEVukhg26LQV42lceWeAr1LKIKyASsN63SUKG2y0cw4j0jxeOOY3MqgM0teJz8kyQGCaPpFEu0kXsblybBfCo+Ta0PZapJYWFCzk+NdiJIK7iy29OqFh7/vFMrdcl6i1iVX4We5Xvjr2XpYoJFd2m2RrUFrU6+vmv3RYYmLJynLI3IGP1jpHU6XZVPukzimvB1ABs6AelwYwUHzlXX/tloA4PuTLhhwUYRIzX948sQUr6Vr26fnZWPHLY/rJ0HyyTPaIUVro+giCdp8rVQoYBKbu+f7UTuN7r1H/XvyofXR6OlFLHi0SdQy91sRr3ER8I6iY19OwkhBOqQMzcpu6DK7A7Lik0J/EOnS1\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);


        VuforiaTrackables ftcPics = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable redTarget1 = ftcPics.get(1);
        redTarget1.setName("RedTarget1");  // tools

        VuforiaTrackable blueTarget1 = ftcPics.get(2);
        blueTarget1.setName("BlueTarget1");  // legos

        VuforiaTrackable redTarget2 = ftcPics.get(3);
        redTarget2.setName("RedTarget2");  // gears

        VuforiaTrackable blueTarget2 = ftcPics.get(0);
        blueTarget2.setName("BlueTarget2");  // wheels

        //For convenience, gather together all the trackable objects in one easily-iterable collection
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(ftcPics);


        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
        //  float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels


        // red target 1
        OpenGLMatrix redTarget1LocationOnField = OpenGLMatrix
        // Then we translate the target off to the RED WALL. Our translation here
        //is a negative translation in X.
        .translation((float) -1803.4, (float) 901.7, 150)
                .multiplied(Orientation.getRotationMatrix(
                        // First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redTarget1.setLocation(redTarget1LocationOnField);
        RobotLog.ii(TAG, "Red Target1=%s", redTarget1LocationOnField.formatAsTransform());


        // red target 2
        OpenGLMatrix redTarget2LocationOnField = OpenGLMatrix
        // Then we translate the target off to the RED WALL. Our translation here
        //is a negative translation in X.
        .translation((float) -1803.4, (float) -300.567, 150)
                .multiplied(Orientation.getRotationMatrix(
                        // First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redTarget2.setLocation(redTarget2LocationOnField);
        RobotLog.ii(TAG, "Red Target2=%s", redTarget2LocationOnField.formatAsTransform());

        // blue target 1
        OpenGLMatrix blueTarget1LocationOnField = OpenGLMatrix
        // Then we translate the target off to the RED WALL. Our translation here
        //is a negative translation in X.
        .translation((float) -901.7, (float) 1803.4, 150)
                .multiplied(Orientation.getRotationMatrix(
                        // First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueTarget1.setLocation(blueTarget1LocationOnField);
        RobotLog.ii(TAG, "Blue Target1=%s", blueTarget1LocationOnField.formatAsTransform());

        // blue target 2
        OpenGLMatrix blueTarget2LocationOnField = OpenGLMatrix
        // Then we translate the target off to the RED WALL. Our translation here
        //is a negative translation in X.
        .translation((float) 300.567, (float) 1803.4, 150)
                .multiplied(Orientation.getRotationMatrix(
                        // First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueTarget2.setLocation(blueTarget2LocationOnField);
        RobotLog.ii(TAG, "Blue Target2=%s", blueTarget2LocationOnField.formatAsTransform() );


        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth / 2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", phoneLocationOnRobot.formatAsTransform());


        ((VuforiaTrackableDefaultListener) redTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) redTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) blueTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener) blueTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);



        //Wait for the game to begin
    //    telemetry.addData(">", "Press Play to start tracking");

        //Start tracking the data sets we care about.
        ftcPics.activate();

    }

    private void changeState(State newState)
    {
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        state = newState;
    }

    //  public void start() {}

    // count number of loops
    int count = 0;

    @Override
    public void loop() {
        OpenGLMatrix robotLocationTransform = null;

        for (VuforiaTrackable trackable : allTrackables) {

    //        telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
                VectorF trans = lastLocation.getTranslation();
                Orientation rot = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                robotX = trans.get(0);
                robotY = trans.get(1);
                robotBearing = rot.thirdAngle;

            }
        }

        //Provide feedback as to where the robot was last located (if we know).

        if (lastLocation != null) {
 //             RobotLog.vv(TAG, "robot=%s", format(lastLocation));
 //             telemetry.addData("Pos", lastLocation.formatAsTransform());
        } else {
 //           telemetry.addData("Pos", "Unknown");
        }
        telemetry.clearAll();
        telemetry.addData("1", String.format("Left Front: %5d  Right Front: %5d ",
                motorRightFront.getCurrentPosition(),
                motorLeftFront.getCurrentPosition()));

        sensorInput = lineSensor.getLightDetected();
        telemetry.addData("sensor", sensorInput);
        telemetry.addData("correction", correction);

        sensorInput = lineSensor.getLightDetected();
        correction = perfectColorValue - sensorInput;


        switch(state) {
            case INITIALIZE:




                // wait 1 hardware cycle
                // motorRightFront.waitOneFullHardwareCycle();
                // motorLeftFront.waitOneFullHardwareCycle();
                // motorRightRear.waitOneFullHardwareCycle();
                // motorLeftRear.waitOneFullHardwareCycle();

                // if both encoders are close to 0, start moving and change state to next state
                if (count >= 1000) {

                    ballFlipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ballFlipper.setTargetPosition(2240);
                    ballFlipper.setPower(1.0);
                    changeState(State.LAUNCH_BALLS);
                }
                else {
                    // continue printing telemetry data to the phone (in main loop)
                    count++;
                    telemetry.addData("0", String.format("State: INITIALIZE"));
                }


            case LAUNCH_BALLS:
                // if ball flipper is done, start moving and change state to next state
                if (ballFlipper.getCurrentPosition() >= 2240) {
                    ballFlipper.setPower(0);
                    motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorLeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorRightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorRightFront.setTargetPosition(3400);
                    motorLeftFront.setTargetPosition(-3400);
                    //crab right
                    motorLeftFront.setPower(-1);
                    motorLeftRear.setPower(1);
                    motorRightFront.setPower(1);
                    motorRightRear.setPower(-1);
                    /*
                    //move forward
                    motorRightFront.setPower(.5);
                    motorRightRear.setPower(.5);
                    motorLeftFront.setPower(.5);
                    motorLeftRear.setPower(.5);*/

                    changeState(State.MOVE_TO_BEACON);
                }
                else {
                    telemetry.addData("0", String.format("State: LAUNCH_BALL"));
                }

            case MOVE_TO_BEACON:

                if ((motorRightFront.getCurrentPosition() >= 3400) && (motorLeftFront.getCurrentPosition() <= -3400)) {
 //                   telemetry.addData("0", "The eagle has flown.");
                    motorRightFront.setPower(0);
                    motorRightRear.setPower(0);
                    motorLeftRear.setPower(0);
                    motorLeftFront.setPower(0);
                    //                  motorRightFront.setPower(-0.25);
  //                  motorLeftFront.setPower(0.25);
  //                  motorRightRear.setPower(-0.25);
  //                  motorLeftRear.setPower(0.25);
  //                  changeState(State.TURN_TO_BEACON);

                    //reset encoders
                    motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    //run all motors without encoders
                    motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorLeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorRightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  //                  motorRightFront.setTargetPosition(1500);
  //                  motorLeftFront.setTargetPosition(1500);
                    //start turning left
                    motorRightFront.setPower(-0.15);
                    motorLeftFront.setPower(0.15);
                    motorRightRear.setPower(-0.15);
                    motorLeftRear.setPower(0.15);
                    changeState(State.TURN_TO_BEACON);
                }
                else {
                    telemetry.addData("0", String.format("State: MOVE_TO_BEACON"));
                }


           case TURN_TO_BEACON:

               if (robotLocationTransform != null) {
                    //STOP
                   motorRightFront.setPower(-.8);
                   motorLeftFront.setPower(-.8);
                   motorRightRear.setPower(-.8);
                   motorLeftRear.setPower(-.8);
                   changeState(State.DRIVE_TO_LINE);

               }
               else {
                   telemetry.addData("0", String.format("State: TURN_TO_BEACON"));
               }


            case DRIVE_TO_LINE:

                if(sensorInput > GRAY_VALUE){
                    speed = 0.5;
        /*            motorRightFront.setPower(0);
                    motorLeftFront.setPower(0);
                    motorRightRear.setPower(0);
                    motorLeftRear.setPower(0);
                    changeState(State.LINE_FOLLOW);
                }
                else{
                    telemetry.addData("0", String.format("State: DRIVE_TO_LINE"));
                    telemetry.addData("0", sensorInput);
                }

            case LINE_FOLLOW:
            // follow the right edge of the line (red team)


               if (correction > 0) { //if it sees white
                   //turn right
                   motorRightFront.setPower(0);
                   motorLeftFront.setPower(-speed);
                   motorRightRear.setPower(0);
                   motorLeftRear.setPower(-speed);

    //               if(!theJudgment){ //if it was too gray before
    //                   speed = speed * -1; //reverse the speed
    //               }
    //               theJudgment = true; //means too white


               }
               else if (correction < 0)  { //if it sees gray
                   //turn left
                   motorRightFront.setPower(-speed);
                   motorLeftFront.setPower(0);
                   motorRightRear.setPower(-speed);
                   motorLeftRear.setPower(0);
    //               if(theJudgment){ //if was too white before
    //                   speed = speed * -1; //reverse the speed
    //               }
    //               theJudgment = false;

               }




                   telemetry.addData("0", String.format("State: DRIVE_TO_LINE"));
                   telemetry.addData("sensor", sensorInput);
                //if range sensor says we are close
       */             motorRightFront.setPower(0);
                    motorLeftFront.setPower(0);
                    motorLeftRear.setPower(0);
                    motorRightRear.setPower(0);
                    changeState(State.STOP);}

            case STOP:
                telemetry.addData("0", String.format("State: STOP"));
                break;
        }
    }

    // set up path segments to get to beacons
    // draw state transition diagram


    //--------------------------------------------------------------------------
    // resetDriveEncoders()
    // Reset both drive motor encoders, and clear current encoder targets.
    //--------------------------------------------------------------------------
    public void resetDriveEncoders()
    {
        //need this?   setEncoderTarget(0, 0);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //--------------------------------------------------------------------------
    // setDriveMode ()
    // Set both drive motors to new mode if they need changing.
    //--------------------------------------------------------------------------
    public void setDriveMode(DcMotor.RunMode mode)
    {
        // Ensure the motors are in the correct mode.
        if (motorLeftFront.getMode() != mode)
            motorLeftFront.setMode(mode);

        if (motorRightFront.getMode() != mode)
            motorRightFront.setMode(mode);

        if (motorLeftRear.getMode() != mode)
            motorLeftRear.setMode(mode);

        if (motorRightRear.getMode() != mode)
            motorRightRear.setMode(mode);
    }


}
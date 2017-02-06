/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

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
import org.firstinspires.ftc.teamcode.HardwareClass;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;

/*
LinearOpMode autonomous
Made by Katie on 2/1/17
Problem: I don't really know what I'm doing because everything I've done before is in state machine format, so I'm just kind of winging it...enjoy
Also, I don't really know how to set one of these up
*/
@Autonomous(name="LinearOpMode") //copied/pasted that from one of our other .javas...?

public class LinearOpMode extends HardwareClass {
    public double perfectColorValue = .05;
    public double sensorInput;
    public double correction;

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;

    public double robotX;
    public double robotY;
    public double robotBearing;
    public double speed;
    public List<VuforiaTrackable> allTrackables;
    private double GRAY_VALUE = 0.02; // gray values never go above .02, so this is the threshold
    // white value is around .08

    public boolean theJudgment; //true means correcting from too white, false means correcting from too gray
    @Override

    public void init() {
         //I think I need to implement the hardware class somewhere so it knows what ballFlipper is
     //   if (something) {
        // ballFlipper.do something;  //also, I'm not sure what this is supposed to do, just that it utilizes the ballFlipper in some way
        // }
        //ALSO: we need to make it so that the ball flipper uses a joystick, not a button, so the user controls how much and when it moves
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


            // reset encoder target positions to 0 on drive wheels and ball flipper
            //resetDriveEncoders();
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
    public void loop(){
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
        }
        else {
            telemetry.addData("0", String.format("State: MOVE_TO_BEACON"));
        }
        /*if (robotLocationTransform != null) {
            //STOP
            motorRightFront.setPower(-.8);
            motorLeftFront.setPower(-.8);
            motorRightRear.setPower(-.8);
            motorLeftRear.setPower(-.8);


        }
        else {
            telemetry.addData("0", String.format("State: TURN_TO_BEACON"));
        }*/
    }

}




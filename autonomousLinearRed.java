/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.State;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Linear Autonomous RED")

public class autonomousLinearRed extends LinearOpMode {

    public DcMotor motorRightRear;
    public DcMotor motorRightFront;
    public DcMotor motorLeftRear;
    public DcMotor motorLeftFront;
    public DcMotor ballFlipper;
    public DcMotor sweeper;
    public Servo buttonPusher;
    public DcMotor ballElevator;
    public Servo theBouncer;
    //public Servo ballGate;
    public OpticalDistanceSensor lineSensor;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public ColorSensor colorSensor;


    public double perfectColorValue = .05;
    public double sensorInput;
    public double correction;
    private double GRAY_VALUE = 0.02; // gray values never go above .02, so this is the threshold
    // white value is around .08
    public boolean theJudgment; //true means correcting from too white, false means correcting from too gray

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //INIT STUFF GOES HERE:

        colorSensor = hardwareMap.colorSensor.get("csense");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rsense");
        lineSensor = hardwareMap.opticalDistanceSensor.get("lsense");
        motorRightFront = hardwareMap.dcMotor.get("rf");
        motorRightRear = hardwareMap.dcMotor.get("rb");

        motorLeftFront = hardwareMap.dcMotor.get("lf");
        motorLeftRear = hardwareMap.dcMotor.get("lb");


        ballFlipper = hardwareMap.dcMotor.get("flip");
        ballElevator = hardwareMap.dcMotor.get("elevator");

        //set directions of motors when driving
        motorLeftRear.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightRear.setDirection(DcMotor.Direction.REVERSE);


        // reset encoder target positions to 0 on drive wheels and ball flipper

        ballFlipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set drive power to 0
        motorRightFront.setPower(0);
        motorLeftFront.setPower(0);
        motorRightRear.setPower(0);
        motorLeftRear.setPower(0);

        telemetry.addData("1", String.format("Left Front: %5d  Right Front: %5d ",
                motorRightFront.getCurrentPosition(),
                motorLeftFront.getCurrentPosition()));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sleep(10000);
        // METHODS GO HERE:
        launchBall1();
        rollBall();
        launchBall2();
        //diddlyTurn();
        //moveToSquare();
        //diddlyTurn2();
       // moveToBeacon1();
       // gettingCloser();
       // pickAColor();
       // hitBeacon();
       // moveBetweenBeacons();
       // pickAColor();
       // hitBeacon();
        end();
    }


    public void launchBall1() {
        // start motors to launch ball
        ballFlipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ballFlipper.setTargetPosition(1120);
        ballFlipper.setPower(1.0);
        //while (encoders <= some value)
        while ( ballFlipper.getCurrentPosition() < 1120) {
            telemetry.addData("0", String.format("loading ball 1, please stand by..."));
            telemetry.update();
        }
        ballFlipper.setPower(0);
        int count=0;
        while (count <= 10000) {
            telemetry.addData("0", String.format("waiting, please stand by..."));
            telemetry.update();
            count++;
        }
        ballFlipper.setTargetPosition(2240);
        ballFlipper.setPower(1);
        while ( ballFlipper.getCurrentPosition() < 2240) {
            telemetry.addData("0", String.format("launching ball 1, please stand by..."));
            telemetry.update();
        }

    }

    public void rollBall(){
        ballElevator.setPower(1);
        int count = 0;
        while (count <= 100000) {
            telemetry.addData("0", String.format("elevating ball 2, please stand by..."));
            telemetry.update();
            count++;
        }
        ballElevator.setPower(0);
    }

    public void launchBall2(){
        // start motors to launch ball
        ballFlipper.setTargetPosition(2900);
        ballFlipper.setPower(1.0);
        while ( ballFlipper.getCurrentPosition() < 2900) {
            telemetry.addData("0", String.format("loading ball 2, please stand by..."));
            telemetry.update();
        }
        ballFlipper.setPower(0);
        int count=0;
        while (count <= 70000) {
            telemetry.addData("0", String.format("waiting, please stand by..."));
            telemetry.update();
            count++;
        }
        ballFlipper.setTargetPosition(4480);
        ballFlipper.setPower(1);
        while ( ballFlipper.getCurrentPosition() < 4480) {
            telemetry.addData("0", String.format("launching ball 2, please stand by..."));
            telemetry.update();
        }
    }

    public void diddlyTurn() {
        //turn around to face beacon
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setTargetPosition(-2000);
        motorRightFront.setPower(-1);
        motorRightRear.setPower (-1);
        while(motorRightFront.getCurrentPosition() > -2000){
            telemetry.addData("0", String.format("we're diddly doing it, please stand by..."));
            telemetry.update();
        }


    }
    public void moveToSquare() {

        //move forward until it gets to the place
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setTargetPosition(-7000);
        motorLeftFront.setTargetPosition(-3500);
        motorRightFront.setPower(-1);
        motorRightRear.setPower(-1);
        motorLeftRear.setPower(-1);
        motorLeftFront.setPower(-1);

        while (motorLeftFront.getCurrentPosition() > -3500 && motorRightFront.getCurrentPosition() > -7000) {
            telemetry.addData("0", String.format("Moving to square, please stand by..."));
            telemetry.update();
        }
        motorRightFront.setPower(0);
        motorRightRear.setPower(0);
        motorLeftRear.setPower(0);
        motorLeftFront.setPower(0);
    }

    /* commenting out everything after we hit the square

    public void diddlyTurn2() {
        //turn around to face beacon
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setTargetPosition(-8500);
        motorRightFront.setPower(-1);
        motorRightRear.setPower (-1);
        while(motorRightFront.getCurrentPosition() > -8500){
            telemetry.addData("0", String.format("we're diddly doing it again, please stand by..."));
            telemetry.update();
        }
    }
    */

    /*
    public void moveToBeacon1() {

        // keep going backwards
        motorRightFront.setTargetPosition(-10500);
        motorLeftFront.setTargetPosition(-6000);

        motorRightFront.setPower(-1);
        motorRightRear.setPower(-1);
        motorLeftRear.setPower(-1);
        motorLeftFront.setPower(-1);

        while (motorLeftFront.getCurrentPosition() > -6000 && motorRightFront.getCurrentPosition() > -10500) {
            telemetry.addData("0", String.format("Moving to beacon, please stand by..."));
            telemetry.update();
        }
    }
    */

    /*
    public void gettingCloser(){
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setPower(-.4);
        motorRightRear.setPower(-.4);
        motorLeftRear.setPower(-.4);
        motorLeftFront.setPower(-.4);
        while(rangeSensor.getDistance(DistanceUnit.CM) > 20) {
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        end();

    }
    */

    /*
    public void pickAColor(){

        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.update();
        int redValue = colorSensor.red();
        int greenValue = colorSensor.green();
        int blueValue = colorSensor.blue();
        boolean isRed = redValue > blueValue && redValue > greenValue;
        boolean isBlue = blueValue > redValue && blueValue > greenValue;


        //if we are too far left crab right
        while(!isRed || !isBlue) {
            motorRightFront.setPower(.5);
            motorLeftRear.setPower(.5);
            motorRightRear.setPower(-.5);
            motorLeftFront.setPower(-.5);
            isRed = redValue > blueValue && redValue > greenValue;
            isBlue = blueValue > redValue && blueValue > greenValue;
        }


        if(isRed){
            pushRight();
        }
        if (isBlue){
            pushLeft();
        }

        //if blue, then call method push left

    } */


    /*
    public void pushRight(){

        // REPLACE THESE WHILE TRUE STATEMENTS WITH ACTUAL INSTRUCTIONS TO CRAB AND PUSH!
        //RIGHT: the right front and left rear wheels must go forward
           //and the right rear and left front wheels must go backward

        //the values of the encoders can be adjusted to a set value, since the robot will end up in the same place
        //and also the color sensor is in the middle of the robot, so it should square up to that and then decide
        //whether to push left or push right, based on a set position
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setTargetPosition(-50);
        motorLeftFront.setTargetPosition(50);
        while(motorRightFront.getCurrentPosition() > -50 && motorLeftFront.getCurrentPosition() < 50) {
            motorRightFront.setPower(-.5);
            motorLeftRear.setPower(-.5);
            motorRightRear.setPower(.5);
            motorLeftFront.setPower(.5);
        }
        while(true){
            telemetry.addData("0",String.format("it's red!"));
        }
    }
    */

    /*
    public void pushLeft(){

        // LEFT: the right rear and left front wheels must go forward
         //  and the right front and left rear wheels must go backward
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setTargetPosition(50);
        motorLeftFront.setTargetPosition(-50);
        while(motorRightFront.getCurrentPosition() < 50 && motorLeftFront.getCurrentPosition() > -50){
            motorRightFront.setPower(.5);
            motorLeftFront.setPower(-.5);
            motorRightRear.setPower(-.5);
            motorLeftRear.setPower(.5);
        }
        while(true){
            telemetry.addData("0",String.format("it's blue!"));
        }
    }
    */

    /*
    public void moveBetweenBeacons(){
        //crab right
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setTargetPosition(-2500);
        motorLeftFront.setTargetPosition(2500);
        motorRightFront.setPower(-1);
        motorRightRear.setPower(1);
        motorLeftRear.setPower(-1);
        motorLeftFront.setPower(1);

        while (motorLeftFront.getCurrentPosition() < 2500 && motorRightFront.getCurrentPosition() > -2500) {
            telemetry.addData("0", String.format("Crabbing to beacon, please stand by..."));
            telemetry.update();
        }
    }
    */

    /*
    public void hitBeacon(){
        telemetry.addData("0", String.format("Pushing the button, please stand by..."));
        telemetry.update();
        //How should we make it hit the beacon? Like, drive into it? Or do we have a button-pusher?

        //drive in to hit the beacon
        while(rangeSensor.getDistance(DistanceUnit.CM) > 6.5){
            motorRightFront.setPower(-.5);
            motorRightRear.setPower(-.5);
            motorLeftRear.setPower(-.5);
            motorLeftFront.setPower(-.5);
        }

        sleep(5000);

        while(rangeSensor.getDistance(DistanceUnit.CM) < 20){
            motorRightFront.setPower(.5);
            motorRightRear.setPower(.5);
            motorLeftRear.setPower(.5);
            motorLeftFront.setPower(.5);
        }




        //it'll drive until the range sensor is a certain distance away from the beacon, at which point the beacon will be pressed
        //and then the robot will stop and go into next function, or end
    }
    */


    public void end(){
        motorRightFront.setPower(0);
        motorRightRear.setPower(0);
        motorLeftRear.setPower(0);
        motorLeftFront.setPower(0);
        ballElevator.setPower(0);
    }


}
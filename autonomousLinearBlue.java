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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.State;

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

@Autonomous(name="Linear Autonomous BLUE")


public class autonomousLinearBlue extends LinearOpMode {

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

        // METHODS GO HERE:
        sleep(10000);
        //waitplz();
        launchBall1();
        rollBall();
        launchBall2();
        //diddlyTurn1();
        //diddlyTurn2();
          //moveToSquare();
        //moveToLine();
        //moveBetweenBeacons();
        //hitBeacon();
        end();
    }

    public void waitplz() {
        int count = 0;
        while (count <= 500000) {
            telemetry.addData("0", String.format("waiting, please stand by..."));
            telemetry.update();
            count++;
        }
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

    public void diddlyTurn1() {
        //turn around to face beacon
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setTargetPosition(-2200);
        motorLeftFront.setPower(-1);
        motorLeftRear.setPower (-1);
        while(motorLeftFront.getCurrentPosition() < -2200){
            telemetry.addData("0", String.format("we're diddly doing it, please stand by..."));
            telemetry.update();
        }
    }

    public void diddlyTurn2() {
        //turn around to face beacon
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setTargetPosition(2400);
        motorRightFront.setPower(1);
        motorRightRear.setPower (1);
        while(motorRightFront.getCurrentPosition() < 2400){
            telemetry.addData("0", String.format("we're diddly doing it, please stand by..."));
            telemetry.update();
        }
    }

    public void moveToSquare() {

        //move forward until it gets to the place
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setTargetPosition(7500);
        motorLeftFront.setTargetPosition(3500);
        motorRightFront.setPower(1);
        motorRightRear.setPower(1);
        motorLeftRear.setPower(1);
        motorLeftFront.setPower(1);

        while (motorLeftFront.getCurrentPosition() < 3500 && motorRightFront.getCurrentPosition() < 7500) {
            telemetry.addData("0", String.format("Moving to beacon, please stand by..."));
            telemetry.update();
        }
    }
/*
    // THIS IS STILL IN PROGRESS......IT IS COPIED AND PASTED FROM autonomousLinearRed
    // moves the robot from the center square to the beacon
    public void moveToLine() {

        // keep going backwards

        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRightFront.setTargetPosition(-3500);
        motorLeftFront.setTargetPosition(-3500);

        motorRightFront.setPower(-1);
        motorRightRear.setPower(-1);
        motorLeftRear.setPower(-1);
        motorLeftFront.setPower(-1);

    }

    public void moveBetweenBeacons(){
        //crab left or right
        motorRightFront.setTargetPosition(-2500);
        motorLeftFront.setTargetPosition(-1500);
        motorRightFront.setPower(-1);
        motorRightRear.setPower(1);
        motorLeftRear.setPower(-1);
        motorLeftFront.setPower(1);

        while (motorLeftFront.getCurrentPosition() < -1500 && motorRightFront.getCurrentPosition() > -2500) {
            telemetry.addData("0", String.format("Crabbing to beacon, please stand by..."));
            telemetry.update();
        }
    }
*/

  /*  public void hitBeacon(){
        telemetry.addData("0", String.format("Pushing the button, please stand by..."));
        telemetry.update();
        //How should we make it hit the beacon? Like, drive into it? Or do we have a button-pusher?
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
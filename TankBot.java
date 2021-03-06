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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareClass;

import static java.lang.Thread.sleep;

/*
TeleOp Mode
Enables control of the robot via the gamepad
*/
@TeleOp(name="TankBot", group ="Tank")


public class TankBot extends HardwareClass {

    // if robot is lopsided, weight adjustment value will compensate
    // RANGE [0, 1] as a percentage of the total power we want.
    // increase adjustment values for wheels that are on the heavier part of the robot
    // because we are scaling weightAdjust according to the power

// ModernRoboticsI2cColorSensor colorSensor;

    ///////CHECK EUCT, JENNA GETS A MESSAGE EVERY TIME//////////////
	@Override
	public void init() {

        motorRightFront = hardwareMap.dcMotor.get("rf");
        motorRightRear = hardwareMap.dcMotor.get("rb");

        motorLeftFront = hardwareMap.dcMotor.get("lf");
        motorLeftRear = hardwareMap.dcMotor.get("lb");

        ballElevator = hardwareMap.dcMotor.get("elevator");
        try{
        ballFlipper = hardwareMap.dcMotor.get("flip");
        sweeper = hardwareMap.dcMotor.get("sweep");

        buttonPusher = hardwareMap.servo.get("button");
        youliftBro = hardwareMap.servo.get("shelf");

        theBouncer = hardwareMap.servo.get("bouncer");
        lineSensor = hardwareMap.opticalDistanceSensor.get("lsense");
        colorSensor = hardwareMap.colorSensor.get("csense");
       // rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rsense");
        colorSensor.enableLed(false);


        //set directions of motors when driving
        motorLeftRear.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightRear.setDirection(DcMotor.Direction.REVERSE);


        ballFlipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
        catch (IllegalArgumentException i){}
        catch (NullPointerException n){}
        // buttonPusher add something here maybe

      /*  I2cDeviceSynch colorCreader = new I2cDeviceSynchImpl(colorSensor, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
            colorCreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        //Passive - For measuring ambient light, eg. the FTC Color Beacon
      */
    }

  //  long timer = 0;
    boolean loaded = false;
	@Override
	public void loop() {

        // adds telemetry data --> shows values of motor positions
        telemetry.addData("motorLeftFront", motorLeftFront.getCurrentPosition());
        telemetry.addData("motorRightFront", motorRightFront.getCurrentPosition());
        // telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
		/*
		 Gamepad 1 controls the motors via the left joystick
		 Gamepad 2 controls nothing right now
        */

        float yVal = gamepad1.left_stick_y; //left joystick controls all wheels
        float xVal = gamepad1.left_stick_x;
        float spinner = gamepad1.right_stick_x; // x axis of the right joystick
        boolean flip = gamepad2.a;
        boolean collectorUp = gamepad2.dpad_up;
        boolean collectorDown = gamepad2.dpad_down;
        boolean fineTunaBallD = gamepad2.right_bumper;
        boolean fineTunaBallU = gamepad2.left_bumper;
        boolean inTheClub = gamepad2.x;
        boolean youWorkOut = gamepad2.y;
        float ballFlippy = gamepad2.left_stick_y;

        float rollers = gamepad2.right_stick_y;


        // negate all values since the y axis is reversed on the joypads and -1 should be 1
        yVal = -(yVal);

        // clip the right/left values so that the values never exceed +/- 1
        yVal = Range.clip(yVal, -1, 1);
        xVal = Range.clip(xVal, -1, 1);
        spinner = Range.clip(spinner, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        yVal = (float) scaleInput(yVal);
        xVal = (float) scaleInput(xVal);
        spinner = (float) scaleInput(spinner);
        int timer = 0;
        // if the a button is pressed


        ballFlippy = (float) scaleInput(ballFlippy);

        if(ballFlippy == 0){
            ballFlipper.setPower(0);
        }
        if(ballFlippy < 0){
            ballFlipper.setPower(-1);
        }
        if(ballFlippy > 0){
            ballFlipper.setPower(1);
        }


       /*if(inTheClub){
           if(theBouncer.getPosition()==0){
               theBouncer.setPosition(1);
           }
           else {
               theBouncer.setPosition(0);
           }
       }*/

        //this code is for the servo bc the one above is spazzy//
       /* if (inTheClub && timer>75) { // if button is pushed
            if (theBouncer.getPosition() >= 0.5) { // if servo position is greater than 0
                // bring the button pusher back in
                theBouncer.setPosition(0.0);
                timer = 0;
            } else { // if servo position is 0
                // deploy the button pusher
                buttonPusher.setPosition(1);
                timer = 0;
            }


        }
        timer = timer+1;

*/


        /*if(fineTunaBallD){
            ballFlipper.setPower(.25);
        }

        else if(fineTunaBallU){
            ballFlipper.setPower(-.25);
        }
        else{

            if (flip) {
                //if the spring is wound
                if (loaded) { //shoot it
                    ballFlipper.setTargetPosition(2200);
                    ballFlipper.setPower(1.0);
                } else { //load it
                    ballFlipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    ballFlipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ballFlipper.setTargetPosition(1540);
                    ballFlipper.setPower(1.0);
                }
            }

            // if motor is at or goes past the target position
            if (loaded) {
                if (ballFlipper.getCurrentPosition() >= 2200) {
                    // stop the motor
                    ballFlipper.setPower(0.0);
                    loaded = false;
                }
            } else {
                if (ballFlipper.getCurrentPosition() >= 1540) {
                    // stop the motor
                    ballFlipper.setPower(0.0);
                    loaded = true;
                }
            }
        }*/

        // reverse the y value because it is backwards on the joystick
        rollers = -rollers;

        // the joystick up makes the rollers go up
        // the joystick down makes the rollers go down

        if(youWorkOut){
            if(youliftBro.getPosition()==0){
                youliftBro.setPosition(1);
            }
            else {
                youliftBro.setPosition(0);
            }
        }

        if(rollers == 0){
            ballElevator.setPower(0);
        }
        else if (rollers > 0) {
            ballElevator.setPower(1);
        }
        else {
            ballElevator.setPower(-1);
        }

    try {




        //   telemetry.addData("counter", timer);
      /*  if (button && timer>75) { // if button is pushed
            if (buttonPusher.getPosition() >= 0.25) { // if servo position is greater than 0
                // bring the button pusher back in
                buttonPusher.setPosition(0.0);
                timer = 0;
            } else { // if servo position is 0
                // deploy the button pusher
                buttonPusher.setPosition(0.5);
                timer = 0;
            }

        }
        timer = timer+1;
*/

        if (collectorUp) {
            sweeper.setPower(1);
        }


        else if (collectorDown) {
            sweeper.setPower(-1);
        }

        else {
            sweeper.setPower(0);
        }


    }
    catch (NullPointerException eagleSeven){}



        // set power to 0 if joysticks are at (0,0)
        if (yVal == 0 && xVal == 0 && spinner == 0) {
            motorRightFront.setPower(0.0);
            motorRightRear.setPower(0.0);
            motorLeftFront.setPower(0.0);
            motorLeftRear.setPower(0.0);
        }

        //Robot can travel forward, backward, right, and left
        // adjust x and y values according to the robot's weight distribution using weightAdjust


        // BACKWARDS: all wheels must go forward
        // the joystick yVal will be positive when going forward
        // in this statement, yVal is negated because the wheels need to go backwards
        if (yVal > Math.abs(xVal)) {
            motorRightFront.setPower(yVal * weightAdjustRF);
            motorRightRear.setPower(yVal * weightAdjustRR);
            motorLeftFront.setPower(yVal * weightAdjustLF);
            motorLeftRear.setPower(yVal * weightAdjustLR);
        }

        // FORWARDS: all wheels must go backward
        // the joystick yVal will be negative when going backward
        // in this statement, yVal is negated because the wheels need to go forwards
        if (yVal < Math.abs(xVal) && Math.abs(yVal) > Math.abs(xVal)) {
            motorRightFront.setPower(yVal * weightAdjustRF * 0.5);
            motorRightRear.setPower(yVal * weightAdjustRR * 0.5);
            motorLeftFront.setPower(yVal * weightAdjustLF * 0.5);
            motorLeftRear.setPower(yVal * weightAdjustLR * 0.5);
        }

        /* RIGHT: the right front and left rear wheels must go forward
           and the right rear and left front wheels must go backward */
        if (xVal > Math.abs(yVal)) {
            motorRightFront.setPower(xVal * weightAdjustRF); // 0 < xVal < 1
            motorRightRear.setPower(-xVal * weightAdjustRR);  // -1 < -xVal < 0
            motorLeftFront.setPower(-xVal * weightAdjustLF);
            motorLeftRear.setPower(xVal * weightAdjustLR);
        }

        /* LEFT: the right rear and left front wheels must go forward
           and the right front and left rear wheels must go backward */
        if (xVal < Math.abs(yVal) && Math.abs(xVal) > Math.abs(yVal)) {
            motorRightFront.setPower(xVal * weightAdjustRF);  // -1 < xVal < 0
            motorRightRear.setPower(-xVal * weightAdjustRR); // 0 < -xVal < 1
            motorLeftFront.setPower(-xVal * weightAdjustLF);
            motorLeftRear.setPower(xVal * weightAdjustLR);
        }


        /* SPIN RIGHT: the left wheels go forward
           and the right wheels go backward */


         /* SPIN LEFT: the right wheels go forward
            and the left wheels go backward */

         if (spinner != 0) {
            // left wheels forward
            motorLeftFront.setPower(-spinner * weightAdjustLF);
            motorLeftRear.setPower(-spinner * weightAdjustLR);
            // right wheels backward
            motorRightFront.setPower(spinner * weightAdjustRF);
            motorRightRear.setPower(spinner * weightAdjustRR);
        }
        else {
        }






    }

	@Override
	public void stop() { // stops all motors when op mode is disabled from driver station
        motorRightFront.setPower(0.0);
        motorRightRear.setPower(0.0);
        motorLeftFront.setPower(0.0);
        motorLeftRear.setPower(0.0);
	}
	
	/*
	 This method scales the joystick input so for low joystick values, the
	 scaled value is less than linear.  This is to make it easier to drive
	 the robot more precisely at slower speeds.
	 */

	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
		
		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		if (index < 0) {
			index = -index;
		} else if (index > 16) {
			index = 16;
		}
		
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}
		
		return dScale;
	}
//defaults to red, but basically this is just a template class
    String redOrBlue() {
        return "red";
    }

}


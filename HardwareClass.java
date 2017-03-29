package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMROpticalDistance;

/**
 * Created by ga on 11/7/16.
 */

public abstract class HardwareClass extends OpMode {
    public DcMotor motorRightRear;
    public DcMotor motorRightFront;
    public DcMotor motorLeftRear;
    public DcMotor motorLeftFront;
    public DcMotor ballFlipper;
    public DcMotor sweeper;
    public Servo buttonPusher;
    public DcMotor ballElevator;
    public Servo theBouncer;
    public Servo youliftBro;

    //public Servo ballGate;
    public OpticalDistanceSensor lineSensor;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public ColorSensor colorSensor;
    public I2cDeviceSynchImpl ledState;



    public float weightAdjustRF = 1;
    public float weightAdjustLR = 1;

    public float weightAdjustRR = (float) 1;
    public float weightAdjustLF = (float) 1;
}

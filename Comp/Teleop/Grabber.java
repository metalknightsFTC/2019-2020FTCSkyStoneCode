package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.io.Serializable;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class Grabber extends LinearOpMode{
    
    private Blinker expansion_Hub_1;
    private Servo grabber1;
    private Servo grabber2;
    private Servo dropper;
    private Servo spin;
    private TouchSensor touch1;
    private TouchSensor touch2;
    private DcMotor rDrive;
    private DcMotor lDrive;
    private DcMotor extend;
    private DcMotor lift;
    
    
     @Override
    public void runOpMode() {
        
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        grabber1 = hardwareMap.get(Servo.class, "grip1");
        grabber2 = hardwareMap.get(Servo.class, "grip2");
        dropper = hardwareMap.get(Servo.class, "drop");
        spin = hardwareMap.get(Servo.class, "spin");
        touch1 = hardwareMap.get(TouchSensor.class, "touch1");
        touch2 = hardwareMap.get(TouchSensor.class, "touch2");
        rDrive = hardwareMap.get(DcMotor.class, "rDrive");
        lDrive = hardwareMap.get(DcMotor.class, "lDrive");
        extend = hardwareMap.get(DcMotor.class, "extend");
        lift = hardwareMap.get(DcMotor.class, "lift");
        
        //init the grabber & its servos
        grabber1.setDirection(Servo.Direction.FORWARD);
        grabber2.setDirection(Servo.Direction.REVERSE);
        spin.setDirection(Servo.Direction.REVERSE);
        //dropper.setDirection(Servo.Direction.FORWARD);

        grabber1.setPosition(0.80);
        grabber2.setPosition(0.80);
        //dropper.setPosition(0.2);
        spin.setPosition(0);
        
        //init all of the motors and set their directions
        lDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        while(opModeIsActive()){
            
            // this actuates the grabbers when a stone is loaded into the grabber assembly
            if(touch1.isPressed() || touch2.isPressed() || (touch1.isPressed() && touch2.isPressed()))
            {
                //chane to make the grabbers close
                grabber1.setPosition(1);
                grabber2.setPosition(1);
                //dropper.setPosition(0.2);
            }//this opens the grabber and drops the the stone
            else if(gamepad2.a)
            { 
                grabber1.setPosition(0.8);
                grabber2.setPosition(0.8);
                //dropper.setPosition(0.5);
            }
            
            //this controls the spin of the of the grabber assembly
            if(gamepad2.x)
            {
                spin.setPosition(0.5);
            }
            else 
            {
                spin.setPosition(0);
            }
            
            //this is the driving of the robot
            
            lDrive.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
            rDrive.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            
            //this controls the extending of the grabber assembly
            extend.setPower(gamepad2.right_stick_y * 0.5);
            
            //this controls the liftind mechanism
            lift.setPower(gamepad2.left_stick_y);
        }

        
    }
}
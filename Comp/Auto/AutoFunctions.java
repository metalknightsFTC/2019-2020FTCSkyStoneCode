/*
Copyright 2019 FIRST Tech Challenge Team 12651
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class AutoFunctions extends LinearOpMode{

        private Blinker Expansion_Hub_1;
        private Blinker Expansion_Hub_2;
        private DcMotor fr;
        private DcMotor br;
        private DcMotor fl;
        private DcMotor bl;
        private DcMotor Rintake;
        private DcMotor Lintake;
        private DcMotor lifter;
        private Servo RintakeServo;
        private Servo LintakeServo;
        private Servo waffleGrabber;
        private Servo waffleGrabber2;
        private Servo pivot;
        private Servo grab;
        private Servo stonePush;
        private Servo skystone;
        private DistanceSensor rightDistance;
        private DistanceSensor leftDistance;
        private TouchSensor grabberTouch;
        private ColorSensor skyStone;
        private ColorSensor lineSensor;
        private DistanceSensor frontDistance;
        private DistanceSensor backDistance;
        private BNO055IMU imu1;
        
        
        
        double lastAngles;
        double globalAngle;
        double power = .30;
        float Yaw_Angle;
        int count;
        int startPos = -1;
        double SkyStonePos;
        
         @Override
    public void runOpMode() {
        
        Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        Expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        imu1 = hardwareMap.get(BNO055IMU.class, "IMU1");
        fl = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "bl");
        Rintake = hardwareMap.get(DcMotor.class, "Rintake");
        Lintake = hardwareMap.get(DcMotor.class, "Lintake");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        RintakeServo = hardwareMap.get(Servo.class, "RintakeServo");
        LintakeServo = hardwareMap.get(Servo.class, "LintakeServo");
        waffleGrabber = hardwareMap.get(Servo.class, "waffleGrabber1");
        waffleGrabber2 = hardwareMap.get(Servo.class, "waffleGrabber2");
        pivot = hardwareMap.get(Servo.class, "pivot");
        grab = hardwareMap.get(Servo.class, "grab");
        stonePush = hardwareMap.get(Servo.class, "stonePush");
        skystone = hardwareMap.get(Servo.class, "skystone");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        grabberTouch = hardwareMap.get(TouchSensor.class, "grabberTouch");
        skyStone = hardwareMap.get(ColorSensor.class, "leftColor");
        lineSensor = hardwareMap.get(ColorSensor.class, "lineSensor");
        backDistance = hardwareMap.get(DistanceSensor.class, "leftColor");
        frontDistance = hardwareMap.get(DistanceSensor.class, "rightColor");
        
        //init all of the motors and set their directions
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        Rintake.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        
        ((DcMotorEx) fr).setTargetPositionTolerance(45);
        ((DcMotorEx) br).setTargetPositionTolerance(45);
        ((DcMotorEx) fl).setTargetPositionTolerance(45);
        ((DcMotorEx) bl).setTargetPositionTolerance(45);
        ((DcMotorEx) lifter).setTargetPositionTolerance(45);
        
        
        //init servos and their starting positions
        RintakeServo.setDirection(Servo.Direction.REVERSE);
        grab.setDirection(Servo.Direction.FORWARD);
        pivot.setDirection(Servo.Direction.FORWARD);
        waffleGrabber.setDirection(Servo.Direction.FORWARD);
        waffleGrabber2.setDirection(Servo.Direction.REVERSE);
        
        
        
        double servoPos = 0;
        double tempSkystonePos = 0;
        /*
        RintakeServo.setPosition(1);
        sleep(500);
        LintakeServo.setPosition(1);
        */
        /*
        RintakeServo.scaleRange(0,1);
        LintakeServo.scaleRange(0,1);
        */
        //grab.setPosition(0.5);
        //pivot.setPosition(0);
        
        waffleGrabber.setPosition(0.25);
        waffleGrabber2.setPosition(0.5);
        
        //actuate the foundation grabber for testing reasons
        /*
        waffleGrabber.setPosition(0);
        waffleGrabber2.setPosition(0);
        */
        IMUInit();
        
        
        //these are me testing to make sure the functions work as intended
       /* moveRotations(0, 1);
        * moveRotations(1, 1);
        * moveRotations(1, -1);
        * moveRotations(2, 1);
        * moveRotations(3, 1);
        * moveAcrossLine(0,0.5);
        */
        
        
        do
        {
            telemetry.addLine("Select Start Position Using Gamepad 1");
            telemetry.addData("Loading Zone", "Press A");
            telemetry.addData("Build Site", "Press Y");
            telemetry.addData("Red", "Press B");
            telemetry.addData("Blue", "Press X");
            telemetry.update();
            
            if(gamepad1.a && gamepad1.b)
            {//Red Loading Zone
                startPos = 0;
            }
            else if (gamepad1.y && gamepad1.b)
            {//Red Build Site
                startPos = 1;
            }
            else if(gamepad1.a && gamepad1.x)
            {//Blue Loading Zone
                startPos = 2;
            }
            else if (gamepad1.y && gamepad1.x)
            {//Blue Build Site
                startPos = 3;
            }
        }
        while (startPos == -1 && !isStopRequested());
        
        
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        skystone.setPosition(0);
        
        RintakeServo.setPosition(0.6);
        //sleep(400);
        LintakeServo.setPosition(0);
        //sleep();
        RintakeServo.setPosition(0);
        
        if(startPos == 1 || startPos == 3)
        {
            moveRotations(0, -0.5);
        }
        
        /*//me testing & calibrating the color function
        while(opModeIsActive()){
            //int c = color();
        //telemetry.addData("color", color());
        //telemetry.update();
        color();
        }*/
        
        
        //moveRotations(0, (int)0.5);
        
        switch (startPos)
        {
            case 0://loading zone Red
                {
                    //moveDistance(1, 3,0.5);
                    
                    waffleGrabber.setPosition(0);
                    waffleGrabber2.setPosition(0);
                    //move back until in range of the quarry
                    moveRotations(0, -1.9);
                    moveRotations(1,0.5);
                    
                    //scan & grab skystone
                    scanSkystone(0.6);
                    moveRotations(0, 1);
                    
                    
                    //move past the Skybirdge & deposite stone
                    //turnIMU(-(getAngle()),0.25);
                    moveAcrossLine(3,1);
                    turnIMU(-(getAngle()),0.25);
                    moveRotations(1,-1);
                    skystone.setPosition(0);
                    sleep(400);
                    moveRotations(0,0.1);
                    
                    
                    
                    /*
                    //move the feild wall
                    moveRotations(1,5.2);
                    turnIMU(-(getAngle()),0.25);
                    moveRotations(1,0.72);
                    turnIMU(-(getAngle()),0.25);
                    moveRotations(0,-1.3);
                    //turnIMU(-(getAngle()),0.25);
                    //moveRotations(1,-0.25);
                    
                    //scan & grab the 2nd Skystone
                    scanSkystone(-0.5);
                    moveRotations(0, 1);
                    //turnIMU(-(getAngle()),0.25);
                    
                    //deposite the 2nd stone
                    //turnIMU(-(getAngle()),0.25);
                    moveAcrossLine(3,1);
                    turnIMU(-(getAngle()),0.25);
                    moveRotations(1,-1);
                    skystone.setPosition(0);
                    sleep(400);
                    moveRotations(0,0.1);
                    */
                    
                    //park under the SkyBridge closest to the feild wall
                    moveAcrossLine(2,0.25);
                    moveRotations(0,0.7);
                    
                }
                break;
            case 1:// build site Red
                {
                    //go and grab the waffle
                    moveRotations(1,-0.25);
                    moveRotations(0,-1.8);
                    waffleGrabber.setPosition(0);
                    waffleGrabber2.setPosition(0);
                    sleep(400);
                    //move waffle to build site & let go of waffle
                    moveRotations(0, 2);
                    //moveRotations(1, 0.25);
                    turnIMU(-90, 1);
                    waffleGrabber.setPosition(0.25);
                    waffleGrabber2.setPosition(0.5);
                    sleep(400);
                    //park the robot under the skybridge
                    moveRotations(3,-2);
                    moveAcrossLine(0,0.5);
                    moveRotations(1,-0.25);
                    
                break;
                }
                case 2://loading zone Blue
                {
                    
                    waffleGrabber.setPosition(0);
                    waffleGrabber2.setPosition(0);
                    //move back until in range of the quarry
                    moveRotations(0, -1.8);
                    //moveDistance(1, 5,0.5);
                    
                    //scan & grab skystone
                    scanSkystone(-0.5);
                    moveRotations(0, 1);
                    
                    
                    //move past the Skybirdge & deposite stone
                    moveAcrossLine(2,1);
                    turnIMU(-(getAngle()),0.25);
                    moveRotations(1,0.7);
                    skystone.setPosition(0);
                    sleep(400);
                    moveRotations(0,0.1);
                    
                    //move the feild wall
                    moveRotations(1,-5.2);
                    turnIMU(-(getAngle()),0.25);
                    moveRotations(1,-0.82);
                    turnIMU(-(getAngle()),0.25);
                    moveRotations(0,-1.3);
                    //moveRotations(1,-0.25);
                    
                    //scan & grab the 2nd Skystone
                    scanSkystone(0.3);
                    moveRotations(0, 1.25);
                    
                    //deposite the 2nd stone
                    moveAcrossLine(2,1);
                    turnIMU(-(getAngle()),0.25);
                    moveRotations(1,0.7);
                    skystone.setPosition(0);
                    sleep(400);
                    moveRotations(0,0.1);
                    
                    //park under the SkyBridge closest to the feild wall
                    moveAcrossLine(3,0.25);
                    
                
                }
                break;
            case 3:// build site Blue
                {
                    //move the lifter out of the way
                    lifterRotations(15);
                    //go and grab the waffle
                    moveRotations(1,0.75);
                    moveRotations(0,-1.78);
                    waffleGrabber.setPosition(0);
                    waffleGrabber2.setPosition(0);
                    sleep(400);
                    //move waffle to build site & let go of waffle
                    moveRotations(0, 1.75);
                    moveRotations(1, -0.25);
                    turnIMU(90, 0.5);
                    moveRotations(1, 0.25);
                    waffleGrabber.setPosition(0.25);
                    waffleGrabber2.setPosition(0.5);
                    sleep(400);
                    //turnIMU(-43, 0.5);
                    //adjust robot so that it parks under the part of the skybridge farthest from the wall
                    moveRotations(0,1.25);
                    lifterRotations(-1.6);
                    //moveRotations(0,-1.5);
                    //park the robot under the skybridge
                    moveRotations(1,-0.5);
                    moveAcrossLine(0,0.5);
                    moveRotations(2,-2);
                    
                break;
                }
            default:
            
        }
        /*
        //these are me testing to make sure the functions work as intended
        moveRotations(0, 1);
        moveRotations(1, 1);
        moveRotations(1, -1);
        //moveRotations(2, 1);
        //moveRotations(3, 1);
        */
        //turnIMU(90, 0.5);
    }
    
    /*
     * this functions moves the robot until a skyStone is seen then grabs it
     */
     public void scanSkystone(double pwr)
     {
         
         fl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
         int num = color();
         while(num != 1 && opModeIsActive())
             {
                 
                  
                 fr.setPower(-pwr);
                 br.setPower(pwr);
                 fl.setPower(pwr);
                 bl.setPower(-pwr);
                 
                 num = color();
                 
             }
             
                 fr.setPower(0);
                 br.setPower(0);
                 fl.setPower(0);
                 bl.setPower(0);
                 
                 //SkyStonePos = ((fr.getCurrentPosition() + br.getCurrentPosition() + fl.getCurrentPosition() + bl.getCurrentPosition()) / 4) / 1120;
                 
                 moveRotations(0,-0.25);
                 sleep(400);
                 skystone.setPosition(1);
                 sleep(490);
                 //moveRotations(0,-0.25);
                 //sleep(100);
                
             
         }
     
    /*
     * move the robot until accros the line
     * direction :
     * 0 = forward
     * 1 = backward
     * 2 = straif to the right
     * 3 = straif to the left
     */
    public void moveAcrossLine(int direction, double pwr){
        
        float Hue = JavaUtil.colorToHue(Color.argb(lineSensor.alpha(), lineSensor.red(), lineSensor.green(),lineSensor.blue()));
        
        fl.setMode(DcMotor.RunMode.RESET_ENCODERS);
        bl.setMode(DcMotor.RunMode.RESET_ENCODERS);
        fr.setMode(DcMotor.RunMode.RESET_ENCODERS);
        br.setMode(DcMotor.RunMode.RESET_ENCODERS);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
         //use a switch to set the motor powers and target positions
        switch(direction){
            case 0://move forward
                {
                    fr.setPower(pwr);
                    br.setPower(pwr);
                    fl.setPower(pwr);
                    bl.setPower(pwr);
                    
                    
                    break;
                }
            case 1://move backward
                {
                    fr.setPower(-pwr);
                    br.setPower(-pwr);
                    fl.setPower(-pwr);
                    bl.setPower(-pwr);
                    
                    
                    break;
                }
            case 2://straif right
                {
                    fr.setPower(-pwr);
                    br.setPower(pwr);
                    fl.setPower(pwr);
                    bl.setPower(-pwr);
                    
                    
                    break;
                }
            case 3://straif left
                {
                    fr.setPower(pwr);
                    br.setPower(-pwr);
                    fl.setPower(-pwr);
                    bl.setPower(pwr);
                    
                    
                    break;
                }
            default:
            {
                fr.setPower(0);
                br.setPower(0);
                fl.setPower(0);
                bl.setPower(0);
                return;
            }
        }
        
        do{
            Hue = JavaUtil.colorToHue(Color.argb(lineSensor.alpha(), lineSensor.red(), lineSensor.green(),lineSensor.blue()));
            
            SkyStonePos = ((fr.getCurrentPosition() + br.getCurrentPosition() + fl.getCurrentPosition() + bl.getCurrentPosition()) / 4) / 1120;
        }
        while(!(Hue > 150 && Hue < 225) && !(Hue > 350 || Hue < 30));
        
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        
        //SkyStonePos = ((fr.getCurrentPosition() + br.getCurrentPosition() + fl.getCurrentPosition() + bl.getCurrentPosition()) / 4) / 1120;
    }
    
    
    /*
     * this function actuates the lifter using rotations
     */
    public void lifterRotations(double rotations)
    {
        rotations = rotations * 1120;
        
        int lifterCurPos = lifter.getCurrentPosition();
        
        lifter.setTargetPosition((int)(lifterCurPos + rotations));
        
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        lifter.setPower(1);
        boolean busy;
        do
        {
            busy = lifter.isBusy();
            telemetry.addData("lifterCurPos", lifterCurPos);

        }
        while(opModeIsActive() && busy);
        
        lifter.setPower(0);
    }
    
    /*
     * this function makes the robot move until a distance from an object in any direction
     * direction: 0 = forward
     *            1 = backward
     *            2 = straif to the right
     *            3 = straif to the left
     * distance: measured in CM tells how far away you should be from an object( the field wall)
     */
     public void moveDistance(int direction, double distance, double pwr){
         
         pwr = Math.abs(pwr);
         
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
         //use a switch to set the motor powers and target positions
        switch(direction){
            case 0://move forward
                {
                    fr.setPower(pwr);
                    br.setPower(pwr);
                    fl.setPower(pwr);
                    bl.setPower(pwr);
                    
                    while(distance < (double)frontDistance.getDistance(DistanceUnit.CM) && opModeIsActive())
                    {
                        telemetry.addData("rightDistance", rightDistance.getDistance(DistanceUnit.CM));
                        telemetry.addData("leftDistance", leftDistance.getDistance(DistanceUnit.CM));
                        telemetry.addData("forwardDistance", frontDistance.getDistance(DistanceUnit.CM));
                        telemetry.addData("backDistance", backDistance.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }
                    break;
                }
            case 1://move backward
                {
                    fr.setPower(-pwr);
                    br.setPower(-pwr);
                    fl.setPower(-pwr);
                    bl.setPower(-pwr);
                    
                    while(distance < (double)backDistance.getDistance(DistanceUnit.CM) && opModeIsActive())
                    {
                        telemetry.addData("rightDistance", rightDistance.getDistance(DistanceUnit.CM));
                        telemetry.addData("leftDistance", leftDistance.getDistance(DistanceUnit.CM));
                        telemetry.addData("forwardDistance", frontDistance.getDistance(DistanceUnit.CM));
                        telemetry.addData("backDistance", backDistance.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }
                    break;
                }
            case 2://straif right
                {
                    fr.setPower(-pwr);
                    br.setPower(pwr);
                    fl.setPower(pwr);
                    bl.setPower(-pwr);
                    
                    while(distance < (double)rightDistance.getDistance(DistanceUnit.CM) && opModeIsActive())
                    {
                        telemetry.addData("rightDistance", rightDistance.getDistance(DistanceUnit.CM));
                        telemetry.addData("leftDistance", leftDistance.getDistance(DistanceUnit.CM));
                        telemetry.addData("forwardDistance", frontDistance.getDistance(DistanceUnit.CM));
                        telemetry.addData("backDistance", backDistance.getDistance(DistanceUnit.CM));
                        telemetry.update();
                    }
                    break;
                }
            case 3://straif left
                {
                    fr.setPower(pwr);
                    br.setPower(-pwr);
                    fl.setPower(-pwr);
                    bl.setPower(pwr);
                    
                    while(distance < (double)leftDistance.getDistance(DistanceUnit.CM) && opModeIsActive())
                    {
                        telemetry.addData("rightDistance", rightDistance.getDistance(DistanceUnit.CM));
                        telemetry.addData("leftDistance", leftDistance.getDistance(DistanceUnit.CM));
                        telemetry.addData("forwardDistance", frontDistance.getDistance(DistanceUnit.CM));
                        telemetry.addData("backDistance", backDistance.getDistance(DistanceUnit.CM)); 
                        telemetry.update();
                    }
                    break;
                }
            default:
            {
                fr.setPower(0);
                br.setPower(0);
                fl.setPower(0);
                bl.setPower(0);
                return;
            }
        }
        
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
     }
    
    /*
     * this function makes the robot move to a position in any direction
     * direction: 0 = forward & backward
     *            1 = straif right & left (- == right && + == left)
     *            2 = diagonal to the right
     *            3 = diagonal to the left
     * rotations: this tells the motors to move to a position
     */
    public void moveRotations(int direction, double rotations){
        rotations = rotations * 1120;
        
        double power = 0.50;
        /*
        fl.setMode(DcMotor.RunMode.RESET_ENCODERS);
        bl.setMode(DcMotor.RunMode.RESET_ENCODERS);
        fr.setMode(DcMotor.RunMode.RESET_ENCODERS);
        br.setMode(DcMotor.RunMode.RESET_ENCODERS);
        */
        int frCurPos = fr.getCurrentPosition();
        int brCurPos = br.getCurrentPosition();
        int flCurPos = fl.getCurrentPosition();
        int blCurPos = bl.getCurrentPosition();
        
        fr.setPower(power);
        br.setPower(power);
        fl.setPower(power);
        bl.setPower(power);
        
        //use a switch to set the motor powers and target positions
        if(direction == 0)
        {
            fr.setTargetPosition((int)(frCurPos + rotations));
            br.setTargetPosition((int)(brCurPos + rotations));
            fl.setTargetPosition((int)(flCurPos + rotations));
            bl.setTargetPosition((int)(blCurPos + rotations));
        }
        else if(direction == 1)
        {
            /*
            fr.setPower(1);
            br.setPower(1);
            fl.setPower(1);
            bl.setPower(1);
            */
            
            fr.setTargetPosition((int)(frCurPos - rotations));
            br.setTargetPosition((int)(brCurPos + rotations));
            fl.setTargetPosition((int)(flCurPos + rotations));
            bl.setTargetPosition((int)(blCurPos - rotations));
        }
        else if(direction == 2)
        {
            fr.setTargetPosition(frCurPos);
            br.setTargetPosition((int)(brCurPos + rotations));
            fl.setTargetPosition((int)(flCurPos + rotations));
            bl.setTargetPosition(blCurPos);
        }
        else if(direction == 3)
        {
            fr.setTargetPosition((int)(frCurPos - rotations));
            br.setTargetPosition(brCurPos);
            fl.setTargetPosition(flCurPos);
            bl.setTargetPosition((int)(blCurPos - rotations));
        }
        
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        boolean motors;
        do
        {
            motors = ((fr.isBusy()||br.isBusy())||(fl.isBusy()||bl.isBusy()));
            telemetry.addData("frCurPos", frCurPos);
            telemetry.addData("brCurPos", brCurPos);
            telemetry.addData("flCurPos", flCurPos);
            telemetry.addData("blCurPos", blCurPos);
        }
        while(opModeIsActive() && motors);
        
        
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
    }
    
    /*
     *This function turns the robot based on the expansion hub IMU
     */
     public void turnIMU(double degrees, double power){
         
      fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        fl.setPower(leftPower);
        fr.setPower(rightPower);
        bl.setPower(leftPower);
        br.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0){
            while (opModeIsActive() && !(getAngle() <= degrees)) {
              // display motor powervalues and the number of times the robot has turned
              telemetry.addData("turn counter", count);
              telemetry.addData("frPwr", fr.getPower());
              telemetry.addData("brPwr", br.getPower());
              telemetry.addData("flPwr", fl.getPower());
              telemetry.addData("blPwr", bl.getPower());
              telemetry.update();
            }
        }
        else    // left turn.
            while (opModeIsActive() && !(getAngle() >= degrees)) {
              // display motor powervalues and the number of times the robot has turned
              telemetry.addData("turn counter", count);
              telemetry.addData("frPwr", fr.getPower());
              telemetry.addData("brPwr", br.getPower());
              telemetry.addData("flPwr", fl.getPower());
              telemetry.addData("blPwr", bl.getPower());
              telemetry.update();
            }

        // turn the motors off.
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        // wait for rotation to stop.
        sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
        
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RESET_ENCODERS);
        bl.setMode(DcMotor.RunMode.RESET_ENCODERS);
        br.setMode(DcMotor.RunMode.RESET_ENCODERS);
        fl.setMode(DcMotor.RunMode.RESET_ENCODERS);
     }
     
     /*
      * this function uses the color sensors to determine which sensor is detecting what
      * this returns a number based on which sensor is detecting which color
      + 
      * it returns:
      *  0 for invalid
      *  1 for both black
      *  2 for both yellow
      */
      private int color(){
      float Hue = JavaUtil.colorToHue(Color.argb(skyStone.alpha(), skyStone.red(), skyStone.green(),skyStone.blue()));

      
      int num = 0;
      
      if(Hue > 90)//black
      {
          num = 1;
      }
      else if((Hue < 90 && Hue >= 40))//yellow
      {
          num = 2;
      }
      

      telemetry.addData("LH", skyStone);
      telemetry.addData("num", num);
      telemetry.update();
      
      return num;
      }
     
       /*
   * function that checks to see if the IMU is calibrated
   */
    private boolean IMU_Calibrated() {
    telemetry.addData("IMU Calibration Status", imu1.getCalibrationStatus());
    telemetry.addData("Gyro Calibrated", imu1.isGyroCalibrated() ? "True" : "False");
    telemetry.addData("System Status", imu1.getSystemStatus().toString());
    return imu1.isGyroCalibrated();
  }

  /*
   * contains all of the IMU initilization rutines
   */
  private void IMUInit() {
    BNO055IMU.Parameters IMU_Parameters;

    IMU_Parameters = new BNO055IMU.Parameters();
    // Set the IMU sensor mode to IMU. This mode uses
    // the IMU gyroscope and accelerometer to
    // calculate the relative orientation of hub and
    // therefore the robot.
    IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
    // Intialize the IMU using parameters object.
    imu1.initialize(IMU_Parameters);
    // Report the initialization to the Driver Station.
    telemetry.addData("Status", "IMU initialized, calibration started.");
    telemetry.update();
    // Wait one second to ensure the IMU is ready.
    sleep(1000);
    // Loop until IMU has been calibrated.
    while (!IMU_Calibrated() && opModeIsActive()) {
      telemetry.addData("If calibration ", "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
      telemetry.update();
      // Wait one second before checking calibration
      // status again.
      sleep(1000);
    }
    // Report calibration complete to Driver Station.
    telemetry.addData("Status", "Calibration Complete");
    telemetry.update();
  }

  /*
   * Resets the cumulative angle tracking to zero.
   */
  private void resetAngle()
  {
      lastAngles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

      globalAngle = 0;
  }

    /*
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        double angles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        double deltaAngle = angles - lastAngles;

        if (deltaAngle < -180){
         deltaAngle += 3160;   
        }
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}

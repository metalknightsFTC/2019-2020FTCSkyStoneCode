package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp

public class Drive extends LinearOpMode{

        private Blinker Expansion_Hub_1;
        private Blinker Expansion_Hub_2;
        private DcMotor fr;
        private DcMotor br;
        private DcMotor fl;
        private DcMotor bl;
        private DcMotor lifter;
        private DcMotor Rintake;
        private DcMotor Lintake;
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
        private ColorSensor SkyStone;
        private DistanceSensor frontDistance;
        private BNO055IMU imu1;
        
        
         @Override
    public void runOpMode() {
        
        Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        Expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        imu1 = hardwareMap.get(BNO055IMU.class, "IMU1");
        fl = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "bl");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        Rintake = hardwareMap.get(DcMotor.class, "Rintake");
        Lintake = hardwareMap.get(DcMotor.class, "Lintake");
        RintakeServo = hardwareMap.get(Servo.class, "RintakeServo");
        LintakeServo = hardwareMap.get(Servo.class, "LintakeServo");
        waffleGrabber = hardwareMap.get(Servo.class, "waffleGrabber1");
        waffleGrabber2 = hardwareMap.get(Servo.class, "waffleGrabber2");
        pivot = hardwareMap.get(Servo.class, "pivot");
        grab= hardwareMap.get(Servo.class, "grab");
        stonePush = hardwareMap.get(Servo.class, "stonePush");
        skystone = hardwareMap.get(Servo.class, "skystone");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        grabberTouch= hardwareMap.get(TouchSensor.class, "grabberTouch");
        SkyStone= hardwareMap.get(ColorSensor.class, "leftColor");
        frontDistance= hardwareMap.get(DistanceSensor.class, "rightColor");
        
        //init all of the motors and set their directions
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        Rintake.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        
        //init servos and their starting positions
        RintakeServo.setDirection(Servo.Direction.REVERSE);
        grab.setDirection(Servo.Direction.REVERSE);
        pivot.setDirection(Servo.Direction.FORWARD);
        waffleGrabber.setDirection(Servo.Direction.FORWARD);
        waffleGrabber2.setDirection(Servo.Direction.REVERSE);
        
        
        double servoPos = 0;
        
        int manOverride = 0;
        int lastPos = 0;
        
        
        /*
        RintakeServo.setPosition(1);
        sleep(500);
        LintakeServo.setPosition(1);
        
        RintakeServo.scaleRange(0,1);
        LintakeServo.scaleRange(0,1);
        */
        //grab.setPosition(0.5);
        //pivot.setPosition(0);
        /*
        waffleGrabber.setPosition(0);
        waffleGrabber2.setPosition(0);
        */
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
        RintakeServo.setPosition(0);
        sleep(500);
        LintakeServo.setPosition(0);
        skystone.setPosition(0);
        
        
        while(opModeIsActive()){
            
        // This is the code that the JABOTs 5223 showed us for dirving with mecanum wheels
       // It has helped the evolution of our robot controls dramatically

       double r = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
       double robotAngle = (Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4);
       double rightX = gamepad1.right_stick_x;
       final double FR = r * Math.sin(robotAngle) - rightX;
       final double FL = r * Math.cos(robotAngle) + rightX;
       final double BR = r * Math.cos(robotAngle) - rightX;
       final double BL = r * Math.sin(robotAngle) + rightX;

       fr.setPower(FR);
       br.setPower(BR);
       fl.setPower(FL);
       bl.setPower(BL);
       
       telemetry.addLine(" Drive motors current positions in the form of rotations");
       telemetry.addData("fr", (fr.getCurrentPosition()/ 1120));
       telemetry.addData("br", (br.getCurrentPosition()/ 1120));
       telemetry.addData("fl", (fl.getCurrentPosition()/ 1120));
       telemetry.addData("bl", (bl.getCurrentPosition()/ 1120));
       telemetry.addLine("Pivoting the Grabber");
       telemetry.addData("0 = Home Position\n1 = 90 Degrees to Robot\n2 = 180 Degrees to robot", manOverride);
       telemetry.addData("rightDistance", rightDistance.getDistance(DistanceUnit.CM));
       telemetry.addData("leftDistance", leftDistance.getDistance(DistanceUnit.CM));
       telemetry.addData("forwardDistance", frontDistance.getDistance(DistanceUnit.CM));
       /*
        * The controls for the robot should be as follows:
        * 
        * left trig = left intake spit out
        * right trig = right intake spit out
        * left bump = left intake suck in 
        * right bump = right intake suck in
        * back = switch between manual & auto for lifter
        * dpadright = intake close
        * dpadleft = intake open
        * back = open waffle grabber
        * x = release block(both manual & auto)
        * b = manual grab block
        * right stick y = manual lifter controls
        * dpadup = pivot grabber 180
        * dpaddown = pivot grabber to home
        */
        /*
        //switch between manual & automatic controls for the lifter
        if(gamepad2.back && lastPos == 0)
        {
            manOverride = 1;
            lastPos = 1;
        }
        else if(gamepad2.back && lastPos == 1)
        {
            manOverride = 2;
            lastPos = 2;
        }
        else if(gamepad2.dpad_down && lastPos == 1)
        {
            manOverride = 0;
            lastPos = 0;
        }
        else if(gamepad2.dpad_down && lastPos == 2)
        {
            manOverride = 1;
            lastPos = 1;
        }
        else if(lastPos == 2 && gamepad2.back)
        {
            manOverride = 0;
            lastPos = 0;
        }
        
        //the automatic controls for pivoting the stones
        switch(manOverride)
        {
            case 0://pivot so the grabber is in the home position
            {
                pivot.setPosition(0);
                break;
            }
                
            case 1://pivot the grabber to a 90 degree position
            {
                pivot.setPosition(0.5);
                break;
            }
                
            case 2://pivot the grabber to a 180 degree position
             {
                 pivot.setPosition(1);
                 break;
             }
            default://if for some reason the imput is more than expected it goes to the home position
            {
                pivot.setPosition(0);
            }
        }
        */
        
        if(gamepad2.b)
        {
            stonePush.setPosition(1);
        }
        else
        {
            stonePush.setPosition(0);
            
        }
        
        if(gamepad2.x)
        {
            skystone.setPosition(1);
        }
        else
        {
            skystone.setPosition(0);
            
        }
        
        if(gamepad2.dpad_down)
        {
            pivot.setPosition(0);
        }
        else if(gamepad2.dpad_up)
        {
            pivot.setPosition(0.69);
            
        }
        //actuate the grabber manualy
        if(gamepad2.y)//release stone
        {
            grab.setPosition(0.5);
            
        }
        else if(gamepad2.a)//grab stone
        {
            grab.setPosition(1);
        }
        
        //actuate the lifter
        lifter.setPower((double)gamepad2.left_stick_y);
        
        //actuate the foundation grabber
        if(gamepad2.back)
        {
            waffleGrabber.setPosition(0.25);
            waffleGrabber2.setPosition(0.5);
            
        }
        else
        {
            waffleGrabber.setPosition(0);
            waffleGrabber2.setPosition(0);
        }
        
        
        //actuate the intake servos
        if(gamepad2.dpad_right)
        {
            RintakeServo.setPosition(0.1);
            LintakeServo.setPosition(0.1);
        }
        else if(gamepad2.dpad_left)
        {
            RintakeServo.setPosition(0);
            LintakeServo.setPosition(0);
        }
        
        //actuate the intake motors
        if(gamepad2.left_bumper)
        {
            Lintake.setPower(1);
        }
        else if(gamepad2.left_trigger == 1)
        {
            Lintake.setPower(-1);
        }
        else
        {
            Lintake.setPower(0);
        }
              
        if(gamepad2.right_bumper)
        {
            Rintake.setPower(-1);
        }
        else if(gamepad2.right_trigger == 1)
        {
            Rintake.setPower(1);
        }
        else
        {
            Rintake.setPower(0);
        }
        
       telemetry.update();
        }
    }
}
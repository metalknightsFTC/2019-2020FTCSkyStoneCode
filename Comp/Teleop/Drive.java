package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
        private DcMotor intakeR;
        private DcMotor intakeL;
        private DcMotor lifterL;
        private DcMotor lifterR;
        private Servo RSkystonePivot;
        private Servo LSkystonePivot;
        private Servo RSkystoneGrab;
        private Servo LSkystoneGrab;
        private Servo rWaffleGrabber;
        private Servo lWaffleGrabber;
        private CRServo extend;
        private Servo grab;
        private TouchSensor limit;
        private DistanceSensor FDistance;
        private DistanceSensor BDistance;
        private TouchSensor RTouch;
        private TouchSensor LTouch;
        private ColorSensor RLine;
        private ColorSensor LLine;
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
        intakeR = hardwareMap.get(DcMotor.class, "intakeR");
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        lifterR = hardwareMap.get(DcMotor.class, "lifterR");
        lifterL = hardwareMap.get(DcMotor.class, "lifterL");
        RSkystonePivot = hardwareMap.get(Servo.class, "RSkystonePivot");
        LSkystonePivot = hardwareMap.get(Servo.class, "LSkystonePivot");
        RSkystoneGrab = hardwareMap.get(Servo.class, "RSkystoneGrab");
        LSkystoneGrab = hardwareMap.get(Servo.class, "LSkystoneGrab");
        rWaffleGrabber = hardwareMap.get(Servo.class, "RWaffleGrabber");
        lWaffleGrabber = hardwareMap.get(Servo.class, "LWaffleGrabber");
        extend = hardwareMap.get(CRServo.class, "extend");
        grab = hardwareMap.get(Servo.class, "grab");
        limit = hardwareMap.get(TouchSensor.class, "limit");
        FDistance = hardwareMap.get(DistanceSensor.class, "FDistance");
        BDistance = hardwareMap.get(DistanceSensor.class, "BDistance");
        RTouch = hardwareMap.get(TouchSensor.class, "RTouch");
        LTouch = hardwareMap.get(TouchSensor.class, "LTouch");
        RLine = hardwareMap.get(ColorSensor.class, "RLine");
        LLine = hardwareMap.get(ColorSensor.class, "LLine");
        BDistance = hardwareMap.get(DistanceSensor.class, "BDistance");
        FDistance = hardwareMap.get(DistanceSensor.class, "FDistance");
        
        
        //init all of the motors and set their directions
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //initilize the servo direction
        grab.setDirection(Servo.Direction.REVERSE);
        RSkystonePivot.setDirection(Servo.Direction.FORWARD);
        LSkystonePivot.setDirection(Servo.Direction.REVERSE);
        RSkystoneGrab.setDirection(Servo.Direction.FORWARD);
        LSkystoneGrab.setDirection(Servo.Direction.REVERSE);
        rWaffleGrabber.setDirection(Servo.Direction.REVERSE);
        lWaffleGrabber.setDirection(Servo.Direction.FORWARD);
        
        //initilize boolean variables to record the previous state of the extender and limit switch
        boolean prevExtendState = false;
        boolean prevLimitState = true;
        //initilize boolean variables to record the previous state of the grabbers
        boolean prevGrabState = false;
        boolean prevSkyGrabStateR = false;
        boolean prevSkyGrabStateL = false;
        //initilize boolean variables to record the previous state of the pivoting arms
        boolean prevSkyPivotStateR = false;
        boolean prevSkyPivotStateL = false;
        //initilize boolean variables to record the previous state of the controler buttons
        boolean prevButtonStateA = false;
        boolean prevButtonStateX = false;
        boolean prevButtonStateB = false;
        boolean prevButtonStateY = false;
        
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        /*
        * The controls for the robot should be as follows:
        * 
        * Gamepad1:
        * 
        * left_stick = directional motion
        * right_stick_x = turn
        * left_bumper = open foundation grabbers
        * 
        * Gamepad2:
        * 
        * dpadright = select right Skystone grabber to be actuated
        * dpadleft = select left Skystone grabber to be actuated
        * x = actuates the left SkyStone arm
        * b = actuates the right SkyStone arm
        * a = grab && release block (defaults to main grabber unless the dpad right or left are being pressed)
        * y = actuates the extender
        * left_stick_y = raise and lower lifter
        * right_stick_y = intake controls
        */
        
        //init servo start position
        
        //up
        LSkystonePivot.setPosition(0.54);
        RSkystonePivot.setPosition(0.49);
        grab.setPosition(0.1);
        //open
      /*  LSkystoneGrab.setPosition(1);
        RSkystoneGrab.setPosition(1);
        sleep(500);
        LSkystonePivot.setPosition(1);
        RSkystonePivot.setPosition(1);*/
        //closed
        LSkystoneGrab.setPosition(0.5);
        RSkystoneGrab.setPosition(0.5);
        
        while(opModeIsActive()){
            
        // This is the code that the JABOTs 5223 showed us for dirving with mecanum wheels
       // It has helped the evolution of our robot controls dramatically

       double r = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
       double robotAngle = (Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4);
       double rightX = gamepad1.right_stick_x;
       final double FR = r * Math.sin(robotAngle) - rightX;
       final double FL = r * Math.cos(robotAngle) + rightX;
       final double BR = r * Math.cos(robotAngle) - rightX;
       final double BL = r * Math.sin(robotAngle) + rightX;


       fr.setPower(FR);
       br.setPower(BR);
       fl.setPower(FL);
       bl.setPower(BL);
       
       //display the motor positions & the robot's distance for debugging purposes
       telemetry.addLine(" Drive motors current positions in the form of rotations");
       telemetry.addData("fr", (fr.getCurrentPosition()/ 1120));
       telemetry.addData("br", (br.getCurrentPosition()/ 1120));
       telemetry.addData("fl", (fl.getCurrentPosition()/ 1120));
       telemetry.addData("bl", (bl.getCurrentPosition()/ 1120));
       telemetry.addData("FDistance", FDistance.getDistance(DistanceUnit.CM));
       telemetry.addData("BDistance", BDistance.getDistance(DistanceUnit.CM));

        //pivot the skystone grabbers
        if(!prevButtonStateB && gamepad2.b && !prevSkyPivotStateR)
        {// right arm down
            RSkystonePivot.setPosition(0.1);
            
            prevSkyPivotStateR = true;
        }
        else if(!prevButtonStateB && gamepad2.b && prevSkyPivotStateR)
        {// right arm up
            RSkystonePivot.setPosition(0.49);
            
            prevSkyPivotStateR = false;
        }
         
        if(!prevButtonStateX && gamepad2.x && !prevSkyPivotStateL)
        {// left arm down
            LSkystonePivot.setPosition(0.15);
            
            prevSkyPivotStateL = true;
        }
        else if(!prevButtonStateX && gamepad2.x && prevSkyPivotStateL)
        {// left arm up
            LSkystonePivot.setPosition(0.54);
            
            prevSkyPivotStateL = false;
        }
        
        
        //actuate the skystone grabbers
        if(!prevButtonStateA && gamepad2.dpad_left && gamepad2.a && !prevSkyGrabStateL)
        {//left close
            LSkystoneGrab.setPosition(0.5);
            prevSkyGrabStateL = true;
        }
         else if(!prevButtonStateA && gamepad2.dpad_left && gamepad2.a && prevSkyGrabStateL)
        {//left open
            LSkystoneGrab.setPosition(0.75);
            prevSkyGrabStateL = false;
        }
        else if(!prevButtonStateA && gamepad2.dpad_right && gamepad2.a && !prevSkyGrabStateR)
        {//right close
            RSkystoneGrab.setPosition(0.5);
            prevSkyGrabStateR = true;
        }
        else if(!prevButtonStateA && gamepad2.dpad_right && gamepad2.a && prevSkyGrabStateR)
        {//right open
            RSkystoneGrab.setPosition(0.75);
            prevSkyGrabStateR = false;
        }
        //actuate the main grabber using the a button on gamepad2
        else if(!prevButtonStateA && !prevGrabState && gamepad2.a)
        {//closed
            grab.setPosition(0.16);
            prevGrabState = true;
        }
        else if(!prevButtonStateA &&prevGrabState && gamepad2.a)
        {//open
            grab.setPosition(0.1);
            prevGrabState = false;
        }
        
        
        //actuate the extender
        if(!prevButtonStateY && !prevExtendState && gamepad2.y)
        {
            extend.setPower(1);
            prevExtendState = true;
            //prevLimitState = false;
        }
        else if(!prevButtonStateY && prevExtendState && gamepad2.y)
        {
            extend.setPower(-1);
            prevExtendState = false;
            
        }
        else if(!prevLimitState && limit.isPressed())
            {//stop when comes neer the end of the extender
                extend.setPower(0);
                //prevLimitState = true;
            }
            
            /*
            //testing to make sure that the magnetic limit switch works
            if(limit.isPressed())
            {//stop when comes neer the end of the extender
                RSkystoneGrab.setPosition(0.75);
                
            }
            else{
                RSkystoneGrab.setPosition(0.5);
            }
            */
            
        //update the current state of the buttons on Gamepad2
            prevButtonStateA = gamepad2.a;
            prevButtonStateX = gamepad2.x;
            prevButtonStateB = gamepad2.b;
            prevButtonStateY = gamepad2.y;
            prevLimitState = limit.isPressed();
        
        
        if(gamepad1.left_bumper)
        {//let go of the foundation using Gamspad1
            rWaffleGrabber.setPosition(0.5);
            lWaffleGrabber.setPosition(0.5);
        }
        if(getRuntime() >= 90){
            //actuate the right foundation grabber if the right touch sensor is actuated
            if(RTouch.isPressed())
            {
                lWaffleGrabber.setPosition(1);
            }
            //actuate the left foundation grabber if the left touch sensor is actuated
            if(LTouch.isPressed())
            {
                rWaffleGrabber.setPosition(1);
            }
        }
        
        
        //actuate the lifter using the left joystick on gamepad2
        lifterR.setPower(gamepad2.left_stick_y);
        lifterL.setPower(gamepad2.left_stick_y);
        
        //actuate the intake using the right joystick on gamepad2
        intakeR.setPower(gamepad2.right_stick_y);
        intakeL.setPower(gamepad2.right_stick_y);

        telemetry.addData("RunTime",getRuntime());
       telemetry.update();
        }
    }
}

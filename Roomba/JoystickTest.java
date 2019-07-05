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
package Roomba;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp

public class JoystickTest extends LinearOpMode {
    private AnalogInput potentiometer;
    private Gyroscope imu;
    private Gyroscope imu_1;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DistanceSensor distance;
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    private DistanceSensor colorSensor;
    private DistanceSensor colorSensor2;
    private Servo servo;
    private TouchSensor touch;
    private TouchSensor magnetic;

    // todo: write your code here
    @Override
    public void runOpMode() {
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        imu_1 = hardwareMap.get(Gyroscope.class, "imu 1");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        colorSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        colorSensor2 = hardwareMap.get(DistanceSensor.class, "colorSensor2");
        touch = hardwareMap.get(TouchSensor.class, "touch");
        magnetic = hardwareMap.get(TouchSensor.class, "magnetic");
        

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //servo.setDirection(Servo.Direction.FORWARD);
        //servo.setPosition(0);
        int direction = 0;
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        while(opModeIsActive()){
            //turn using left joyStick
            frontRight.setPower(-gamepad1.left_stick_x);
            backRight.setPower(-gamepad1.left_stick_x);
            frontLeft.setPower(gamepad1.left_stick_x);
            backLeft.setPower(gamepad1.left_stick_x);
            
            if(gamepad1.left_bumper){
                direction = 1;
            }
            else if(gamepad1.right_bumper){
                direction = 0;
            }
            
            //move in all directions using the right joyStick
            switch (direction){
                case 0:
                    //move forwards & straif
                    frontRight.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x);
                    backRight.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x);
                    frontLeft.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x);
                    backLeft.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x);
                case 1:
                    //move diagonal
                    frontRight.setPower(gamepad1.right_stick_y);
                    backRight.setPower(gamepad1.right_stick_x);
                    frontLeft.setPower(gamepad1.right_stick_x);
                    backLeft.setPower(gamepad1.right_stick_y);
            }
            
            telemetry.addData("direction", direction);
            telemetry.addData("FRi",frontRight.getPower());
            telemetry.addData("Fl",frontLeft.getPower());
            telemetry.addData("BRi",backRight.getPower());
            telemetry.addData("Bl",backLeft.getPower());
            telemetry.update();
        }
        }
    }

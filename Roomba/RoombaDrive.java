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
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import android.graphics.Color;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class RoombaDrive extends LinearOpMode {
    private AndroidSoundPool androidSoundPool;
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
    private ColorSensor colorSensor;
    private ColorSensor colorSensor2;
    private TouchSensor touch;
    private TouchSensor magnetic;
    private Servo servo;


    @Override
    public void runOpMode() {
        androidSoundPool = new AndroidSoundPool();
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
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");
        touch = hardwareMap.get(TouchSensor.class, "touch");
        magnetic = hardwareMap.get(TouchSensor.class, "magnetic");
        
        
        androidSoundPool.initialize(SoundPlayer.getInstance());
        androidSoundPool.setVolume(1F);
        androidSoundPool.play("WiiSports.mp3");
        
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //servo.setDirection(Servo.Direction.FORWARD);
        //servo.setPosition(0);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("magnet", magnetic.isPressed());
            telemetry.addData("touch", touch.isPressed());
            telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("potentiometer voltage", potentiometer.getVoltage());
            telemetry.addData("color", JavaUtil.colorToHue(Color.argb(colorSensor.alpha(), colorSensor.red(), colorSensor.green(), colorSensor.blue())));
        //telemetry.addData("servo position", servo.getPosition());
            
            if(magnetic.isPressed() == true){
                androidSoundPool.setVolume(0);
                
                //servo.setPosition(1);
                telemetry.addLine("hello");
            }
            else{
                //servo.setPosition(0);
                
            }
            
            if(touch.isPressed() == true){
                androidSoundPool.play("r2d2Scream.mp3");
                frontRight.setPower(-1);
                backLeft.setPower(-1);
                sleep(100);
                
            }
            
            if(distance.getDistance(DistanceUnit.CM) <= 5.0){
                androidSoundPool.play("nope.mp3");
                frontRight.setPower(-1);
                backLeft.setPower(-1);
                backRight.setPower(-1);
                frontLeft.setPower(-1);
                sleep(100);
            }
            
            if(gamepad1.dpad_up == true){
                frontRight.setPower(0.5);
                backLeft.setPower(0.5);
            }
            else if(gamepad1.dpad_down == true){
                frontRight.setPower(-0.5);
                backLeft.setPower(-0.5);
            }
            else if(gamepad1.dpad_left == true){
                backRight.setPower(0.5);
                frontLeft.setPower(0.5);
            }
            else if(gamepad1.dpad_right == true){
                backRight.setPower(-0.5);
                frontLeft.setPower(-0.5);
            }
            else if(gamepad1.y == true){
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(1);
                frontLeft.setPower(1);
            }
            else if(gamepad1.a == true){
                frontRight.setPower(-1);
                backLeft.setPower(-1);
                backRight.setPower(-1);
                frontLeft.setPower(-1);
            }
            else if(gamepad1.x == true){
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(-1);
                frontLeft.setPower(-1);
            }
            else if(gamepad1.b == true){
                frontRight.setPower(-1);
                backLeft.setPower(-1);
                backRight.setPower(1);
                frontLeft.setPower(1);
            }
            else if(gamepad1.left_stick_button == true){
                frontRight.setPower(0.5);
                backLeft.setPower(-0.5);
                backRight.setPower(0.5);
                frontLeft.setPower(-0.5);
            }
            else if(gamepad1.right_stick_button == true){
                frontRight.setPower(-0.5);
                backLeft.setPower(0.5);
                backRight.setPower(-0.5);
                frontLeft.setPower(0.5);
            }
            else{
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                
            }
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}

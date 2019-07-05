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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "RoombaIMU", group = "Roomba")
public class RoombaIMU extends LinearOpMode {

  private DcMotor frontRight;
  private DcMotor backLeft;
  private BNO055IMU imu1;
  private DcMotor backRight;
  private DcMotor frontLeft;
  private BNO055IMU imu;
  private DistanceSensor distance;

  double Left_Power;
  double Right_Power;
  float Yaw_Angle;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    frontRight = hardwareMap.dcMotor.get("frontRight");
    backLeft = hardwareMap.dcMotor.get("backLeft");
    imu1 = hardwareMap.get(BNO055IMU.class, "imu 1");
    backRight = hardwareMap.dcMotor.get("backRight");
    frontLeft = hardwareMap.dcMotor.get("frontLeft");
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    distance = hardwareMap.get(DistanceSensor.class, "distance");

    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Reverse direction of one side so robot moves
    // forward rather than spinning in place.
    backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    // Create an IMU parameters object.
    IMUInit();
    // Wait for Start to be pressed on Driver Station.
    waitForStart();
    
    
    forward(2, 1);
    backward(2, 1);
    turnIMU(90, 0.2);
    right(2, 1);
    left(2, 1);
    /*
    
    
    //doesn't work
    
    while(opModeIsActive()){
        if (gamepad1.dpad_up){
            straif(0, 2);
        }
        else if(gamepad1.dpad_down){
            straif(1, 2);
        }
        else if(gamepad1.dpad_right){
            straif(2, 2);
        }
        else if(gamepad1.dpad_left){
            straif(3, 2);
        }
        else if (gamepad1.x){
            return;
        }
    }
    */
  }

  /**
   * Function that becomes true when gyro is calibrated and
   * reports calibration status to Driver Station in the meantime.
   */
  private boolean IMU_Calibrated() {
    telemetry.addData("IMU Calibration Status", imu1.getCalibrationStatus());
    telemetry.addData("Gyro Calibrated", imu1.isGyroCalibrated() ? "True" : "False");
    telemetry.addData("System Status", imu1.getSystemStatus().toString());
    return imu1.isGyroCalibrated();
  }

  /**
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
    while (!IMU_Calibrated()) {
      telemetry.addData("If calibration ", "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
      telemetry.update();
      // Wait one second before checking calibration
      // status again.
      sleep(1000);
    }
    // Report calibration complete to Driver Station.
    telemetry.addData("Status", "Calibration Complete");
    telemetry.addData("Action needed:", "Please press the start triangle");
    telemetry.update();
  }

  /**
   * turns the robot to a set angle at a set power for each side
   */
  private void turnIMU(double angle, double power) {
    frontRight.setPower(power);
    backLeft.setPower(-power);
    backRight.setPower(power);
    frontLeft.setPower(-power);
    while (!(Yaw_Angle >= angle || isStopRequested())) {
      // Update Yaw-Angle variable with current yaw.
      Yaw_Angle = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      // Report yaw orientation to Driver Station.
      telemetry.addData("Yaw value", Yaw_Angle);
      telemetry.update();
    }
    frontRight.setPower(0);
    backLeft.setPower(0);
    backRight.setPower(0);
    frontLeft.setPower(0);
  }

  /**
   *The robot drives backward untill a position is reached using the frontRight & backLeft motors
   */
  private void backward(double variance, int rotations2) {
    rotations2 = rotations2 * 1120;
    //set the target position for the motors
    frontRight.setTargetPosition(frontRight.getCurrentPosition() - rotations2);
    backLeft.setTargetPosition(backLeft.getCurrentPosition() - rotations2);
    // Initialize motor power variables to 30%.
    Left_Power = -0.3;
    Right_Power = -0.3;
    // Set motor powers to the variable values.
    frontRight.setPower(Right_Power);
    backLeft.setPower(Left_Power);
    // Move robot backward until target position reached or until stop
    // is pressed on Driver Station.
    while (!(frontRight.getCurrentPosition() <= frontRight.getTargetPosition() && backLeft.getCurrentPosition() <= backLeft.getTargetPosition() || isStopRequested())) {
      // Save gyro's yaw angle
      Yaw_Angle = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      // Report yaw orientation to Driver Station.
      telemetry.addData("Yaw angle", Yaw_Angle);
      // If the robot is moving straight ahead the
      // yaw value will be close to zero. If it's not, we
      // need to adjust the motor powers to adjust heading.
      // If robot yaws right or left by 5 or more,
      // adjust motor power variables to compensation.
      if (Yaw_Angle < variance) {
        // Turn left
        Left_Power = -0.25;
        Right_Power = -0.35;
      } else if (Yaw_Angle > -variance) {
        // Turn right.
        Left_Power = -0.35;
        Right_Power = -0.25;
      } else {
        // Continue straight
        Left_Power = -0.3;
        Right_Power = -0.3;
      }
      // Report the position & power levels to the Driver Station.
      telemetry.addData("left position", backLeft.getCurrentPosition() / 1120);
      telemetry.addData("right position", frontRight.getCurrentPosition() / 1120);
      telemetry.addData("Left Motor Power", Left_Power);
      telemetry.addData("Right Motor Power", Right_Power);
      // Update the motors to the new power levels.
      frontRight.setPower(Right_Power);
      backLeft.setPower(Left_Power);
      backRight.setPower(0);
      frontLeft.setPower(0);
      telemetry.update();
      // Wait 1/5 second before checking again.
      sleep(200);
    }
  }

  /**
   * The robot drives forward untill a position is reached using the frontRight & backLeft motors
   */
  private void forward(double variance, int rotations2) {
    rotations2 = rotations2 * 1120;
    // Create a timer object with millisecond
    // resolution and save in ElapsedTime variable.
    frontRight.setTargetPosition(frontRight.getCurrentPosition() + rotations2);
    backLeft.setTargetPosition(backLeft.getCurrentPosition() + rotations2);
    // Initialize motor power variables to 30%.
    Left_Power = 0.3;
    Right_Power = 0.3;
    // Set motor powers to the variable values.
    frontRight.setPower(Right_Power);
    backLeft.setPower(Left_Power);
    // Move robot forward for 2 seconds or until stop
    // is pressed on Driver Station.
    while (!(frontRight.getCurrentPosition() >= frontRight.getTargetPosition() && backLeft.getCurrentPosition() >= backLeft.getTargetPosition() || isStopRequested())) {
      // Save gyro's yaw angle
      Yaw_Angle = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      // Report yaw orientation to Driver Station.
      telemetry.addData("Yaw angle", Yaw_Angle);
      // If the robot is moving straight ahead the
      // yaw value will be close to zero. If it's not, we
      // need to adjust the motor powers to adjust heading.
      // If robot yaws right or left by 5 or more,
      // adjust motor power variables to compensation.
      if (Yaw_Angle < variance) {
        // Turn left
        Left_Power = 0.25;
        Right_Power = 0.35;
      } else if (Yaw_Angle > -variance) {
        // Turn right.
        Left_Power = 0.35;
        Right_Power = 0.25;
      } else {
        // Continue straight
        Left_Power = 0.3;
        Right_Power = 0.3;
      }
      // Report the new power levels to the Driver Station.
      telemetry.addData("left position", backLeft.getCurrentPosition() / 1120);
      telemetry.addData("right position", frontRight.getCurrentPosition() / 1120);
      telemetry.addData("Left Motor Power", Left_Power);
      telemetry.addData("Right Motor Power", Right_Power);
      // Update the motors to the new power levels.
      frontRight.setPower(Right_Power);
      backLeft.setPower(Left_Power);
      backRight.setPower(0);
      frontLeft.setPower(0);
      telemetry.update();
      // Wait 1/5 second before checking again.
      sleep(200);
    }
  }

  /**
   * The robot drives backward untill a position is reached using the backRight & frontLeft motors
   */
  private void left(double variance, int rotations2) {
    rotations2 = rotations2 * 1120;
    // Create a timer object with millisecond
    // resolution and save in ElapsedTime variable.
    backRight.setTargetPosition(backRight.getCurrentPosition() - rotations2);
    frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - rotations2);
    // Initialize motor power variables to 30%.
    Left_Power = -0.3;
    Right_Power = -0.3;
    // Set motor powers to the variable values.
    backRight.setPower(Right_Power);
    frontLeft.setPower(Left_Power);
    // Move robot forward for 2 seconds or until stop
    // is pressed on Driver Station.
    while (!(backRight.getCurrentPosition() <= backRight.getTargetPosition() && frontLeft.getCurrentPosition() <= frontLeft.getTargetPosition() || isStopRequested())) {
      // Save gyro's yaw angle
      Yaw_Angle = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      // Report yaw orientation to Driver Station.
      telemetry.addData("Yaw angle", Yaw_Angle);
      // If the robot is moving straight ahead the
      // yaw value will be close to zero. If it's not, we
      // need to adjust the motor powers to adjust heading.
      // If robot yaws right or left by 5 or more,
      // adjust motor power variables to compensation.
      if (Yaw_Angle < variance) {
        // Turn left
        Left_Power = -0.25;
        Right_Power = -0.35;
      } else if (Yaw_Angle > -variance) {
        // Turn right.
        Left_Power = -0.35;
        Right_Power = -0.25;
      } else {
        // Continue straight
        Left_Power = -0.3;
        Right_Power = -0.3;
      }
      // Report the new power levels to the Driver Station.
      telemetry.addData("left position", frontLeft.getCurrentPosition() / 1120);
      telemetry.addData("right position", backRight.getCurrentPosition() / 1120);
      telemetry.addData("Left Motor Power", Left_Power);
      telemetry.addData("Right Motor Power", Right_Power);
      // Update the motors to the new power levels.
      backRight.setPower(Right_Power);
      frontLeft.setPower(Left_Power);
      frontRight.setPower(0);
      backLeft.setPower(0);
      telemetry.update();
      // Wait 1/5 second before checking again.
      sleep(200);
    }
    // Now let's execute a right turn using power
    // levels that will cause a turn in place.
  }

  /**
   * The robot drives forward untill a position is reached using the backRight & frontLeft motors
   */
  private void right(double variance, int rotations2) {
    rotations2 = rotations2 * 1120;
    // Create a timer object with millisecond
    // resolution and save in ElapsedTime variable.
    backRight.setTargetPosition(backRight.getCurrentPosition() + rotations2);
    frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + rotations2);
    // Initialize motor power variables to 30%.
    Left_Power = 0.3;
    Right_Power = 0.3;
    // Set motor powers to the variable values.
    backRight.setPower(Right_Power);
    frontLeft.setPower(Left_Power);
    // Move robot forward for 2 seconds or until stop
    // is pressed on Driver Station.
    while (!(backRight.getCurrentPosition() >= backRight.getTargetPosition() && frontLeft.getCurrentPosition() >= frontLeft.getTargetPosition() || isStopRequested())) {
      // Save gyro's yaw angle
      Yaw_Angle = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      // Report yaw orientation to Driver Station.
      telemetry.addData("Yaw angle", Yaw_Angle);
      // If the robot is moving straight ahead the
      // yaw value will be close to zero. If it's not, we
      // need to adjust the motor powers to adjust heading.
      // If robot yaws right or left by 5 or more,
      // adjust motor power variables to compensation.
      if (Yaw_Angle < variance) {
        // Turn left
        Left_Power = 0.25;
        Right_Power = 0.35;
      } else if (Yaw_Angle > -variance) {
        // Turn right.
        Left_Power = 0.35;
        Right_Power = 0.25;
      } else {
        // Continue straight
        Left_Power = 0.3;
        Right_Power = 0.3;
      }
      // Report the new power levels to the Driver Station.
      telemetry.addData("left position", frontLeft.getCurrentPosition() / 1120);
      telemetry.addData("right position", backRight.getCurrentPosition() / 1120);
      telemetry.addData("Left Motor Power", Left_Power);
      telemetry.addData("Right Motor Power", Right_Power);
      // Update the motors to the new power levels.
      backRight.setPower(Right_Power);
      frontLeft.setPower(Left_Power);
      frontRight.setPower(0);
      backLeft.setPower(0);
      telemetry.update();
      // Wait 1/5 second before checking again.
      sleep(200);
    }
  }
  
  /*
  
  //doesn't work
  
  private void straif(int direction, double variance){
                
    Yaw_Angle = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

    switch (direction){
        case 0://forwards
        
            frontRight.setPower(0.3);
            backRight.setPower(0.3);
            frontLeft.setPower(0.3);
            backLeft.setPower(0.3);
            
            if(Yaw_Angle > variance){
                frontRight.setPower(0.3);
                backRight.setPower(0.3);
                frontLeft.setPower(0.25);
                backLeft.setPower(0.25);
            }
            else if(Yaw_Angle < -variance){
                frontRight.setPower(0.25);
                backRight.setPower(0.25);
                frontLeft.setPower(0.3);
                backLeft.setPower(0.3);
            }
            
        case 1: //backwards
            
                //Yaw_Angle = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            
            frontRight.setPower(-0.3);
            backRight.setPower(-0.3);
            frontLeft.setPower(-0.3);
            backLeft.setPower(-0.3);
            
            if(Yaw_Angle > variance){
                frontRight.setPower(-0.3);
                backRight.setPower(-0.3);
                frontLeft.setPower(-0.25);
                backLeft.setPower(-0.25);
            }
            else if(Yaw_Angle < -variance){
                frontRight.setPower(-0.25);
                backRight.setPower(-0.25);
                frontLeft.setPower(-0.3);
                backLeft.setPower(-0.3);
            }
            
        case 2:// straif Right
        
            //Yaw_Angle = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        
            frontRight.setPower(-0.3);
            backRight.setPower(0.3);
            frontLeft.setPower(0.3);
            backLeft.setPower(-0.3);
            
            if(Yaw_Angle > variance){
                frontRight.setPower(-0.25);
                backRight.setPower(0.3);
                frontLeft.setPower(0.3);
                backLeft.setPower(-0.25);
            }
            else if(Yaw_Angle < -variance){
                frontRight.setPower(-0.3);
                backRight.setPower(0.25);
                frontLeft.setPower(0.25);
                backLeft.setPower(-0.3);
            }
            
        case 3:// straif left
        
    //Yaw_Angle = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        
            frontRight.setPower(0.3);
            backRight.setPower(-0.3);
            frontLeft.setPower(-0.3);
            backLeft.setPower(0.3);
            
            if(Yaw_Angle > variance){
                frontRight.setPower(0.25);
                backRight.setPower(-0.3);
                frontLeft.setPower(-0.3);
                backLeft.setPower(0.25);
            }
            else if(Yaw_Angle < -variance){
                frontRight.setPower(0.3);
                backRight.setPower(-0.25);
                frontLeft.setPower(-0.25);
                backLeft.setPower(0.3);
            }
    }
    
            telemetry.addData("direction", direction);
            telemetry.addData("yaw", Yaw_Angle);
            telemetry.addData("FRi",frontRight.getPower());
            telemetry.addData("Fl",frontLeft.getPower());
            telemetry.addData("BRi",backRight.getPower());
            telemetry.addData("Bl",backLeft.getPower());
            telemetry.update();
  }
  */
}

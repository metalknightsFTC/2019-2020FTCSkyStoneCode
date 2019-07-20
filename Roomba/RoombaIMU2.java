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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous

public class RoombaIMU2 extends LinearOpMode{
    private AnalogInput potentiometer;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DistanceSensor distance;
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    private ColorSensor colorSensor;
    private ColorSensor colorSensor2;
    private Servo servo;
    private TouchSensor touch;
    private TouchSensor magnetic;
    private BNO055IMU imu1;
    private Gyroscope imu;
    
    double lastAngles;
    double globalAngle, power = .30;
    
    @Override
  public void runOpMode() {
    potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
    imu = hardwareMap.get(Gyroscope.class, "imu");
    imu1 = hardwareMap.get(BNO055IMU.class, "imu 1");
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
    
    frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    IMUInit();
    
    waitForStart();
    
    move(0, 0.5, 1);
    move(0, 0.5, -1);
    move(1, 0.5, 1);
    move(1, 0.5, -1);
    
    turnIMU(90, 0.5);
    turnIMU(-90, 0.5);
    turnIMU(-180, 0.5);
    turnIMU(180, 0.5);

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

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void turnIMU(double degrees, double power)
    {
      frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      
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
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0){
            while (opModeIsActive() && !(getAngle() <= degrees)) {
              // display motor powervalues
              telemetry.addData("frPwr", frontRight.getPower());
              telemetry.addData("brPwr", backRight.getPower());
              telemetry.addData("flPwr", frontLeft.getPower());
              telemetry.addData("blPwr", backLeft.getPower());
              telemetry.update();
            }
        }
        else    // left turn.
            while (opModeIsActive() && !(getAngle() >= degrees)) {
              // display motor powervalues
              telemetry.addData("frPwr", frontRight.getPower());
              telemetry.addData("brPwr", backRight.getPower());
              telemetry.addData("flPwr", frontLeft.getPower());
              telemetry.addData("blPwr", backLeft.getPower());
              telemetry.update();
            }

        // turn the motors off.
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
        
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }
    
    /*
     *move the robot diagonaly using direction: 0 or 1
     *using rotations and power
     */
    private void move(int direction, double power, int rotations ){
      rotations = rotations * 1120;
      
      if(direction == 0){
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + rotations);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + rotations);

        frontRight.setPower(power);
        backLeft.setPower(power);
        
        while(opModeIsActive() && frontRight.isBusy() && backLeft.isBusy()){
          double correction = checkDirection();
          frontRight.setPower(power + correction);
          backLeft.setPower(power - correction);
          
          //display motor power and positions
          telemetry.addData("frPwr", frontRight.getPower());
          telemetry.addData("blPwr", backLeft.getPower());
          telemetry.addData("frPos", frontRight.getCurrentPosition());
          telemetry.addData("blPos", backLeft.getCurrentPosition());
          telemetry.addData("frTarget", frontRight.getTargetPosition());
          telemetry.addData("blTarget", backLeft.getTargetPosition());
          telemetry.update();
        }
        
      }
      else if(direction == 1){
        backRight.setTargetPosition(backRight.getCurrentPosition() + rotations);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + rotations);
        
        backRight.setPower(power);
        frontLeft.setPower(power);
        
        while(opModeIsActive() && backRight.isBusy() && frontLeft.isBusy()){
          double correction = checkDirection();
          backRight.setPower(power + correction);
          frontLeft.setPower(power - correction);
          
          //display motor power and positions
          telemetry.addData("brPwr", backRight.getPower());
          telemetry.addData("flPwr", frontLeft.getPower());
          telemetry.addData("brPos", backRight.getCurrentPosition());
          telemetry.addData("flPos", frontLeft.getCurrentPosition());
          telemetry.addData("brTarget", backRight.getTargetPosition());
          telemetry.addData("flTarget", frontLeft.getTargetPosition());
          telemetry.update();
        }
      }
      else{
        telemetry.addLine("ERROR: Invalid Direction");
      }
    }
    
    /*
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
}

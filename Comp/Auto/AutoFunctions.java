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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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
import java.util.ArrayList;
import java.util.List;


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
  private Servo RSkystonePivot;
  private Servo LSkystonePivot;
  private Servo RSkystoneGrab;
  private Servo LSkystoneGrab;
  private Servo rWaffleGrabber;
  private Servo lWaffleGrabber;
  private DistanceSensor FDistance;
  private DistanceSensor BDistance;
  private DistanceSensor RDistance;
  private DistanceSensor LDistance;
  private TouchSensor RTouch;
  private TouchSensor LTouch;
  private ColorSensor RLine;
  private ColorSensor LLine;
  private ColorSensor RSkyStoneColor;
  private ColorSensor LSkyStoneColor;
  private BNO055IMU imu1;
  
  double lastAngles;
  double globalAngle;
  float Yaw_Angle;
  
  int startPos = -1;
  int endPos = -1;
  int skyStonePos;
  
  @Override
  public void runOpMode() {
    
    Expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
    Expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
    imu1 = hardwareMap.get(BNO055IMU.class, "IMU");
    fl = hardwareMap.get(DcMotor.class, "fr");
    bl = hardwareMap.get(DcMotor.class, "br");
    fr = hardwareMap.get(DcMotor.class, "fl");
    br = hardwareMap.get(DcMotor.class, "bl");
    rWaffleGrabber = hardwareMap.get(Servo.class, "RWaffleGrabber");
    lWaffleGrabber = hardwareMap.get(Servo.class, "LWaffleGrabber");
    RSkystonePivot = hardwareMap.get(Servo.class, "RSkystonePivot");
    LSkystonePivot = hardwareMap.get(Servo.class, "LSkystonePivot");
    RSkystoneGrab = hardwareMap.get(Servo.class, "RSkystoneGrab");
    LSkystoneGrab = hardwareMap.get(Servo.class, "LSkystoneGrab");
    FDistance = hardwareMap.get(DistanceSensor.class, "FDistance");
    BDistance = hardwareMap.get(DistanceSensor.class, "BDistance");
    RDistance = hardwareMap.get(DistanceSensor.class, "RSkyStoneColor");
    LDistance = hardwareMap.get(DistanceSensor.class, "LSkyStoneColor");
    RTouch = hardwareMap.get(TouchSensor.class, "RTouch");
    LTouch = hardwareMap.get(TouchSensor.class, "LTouch");
    RLine = hardwareMap.get(ColorSensor.class, "RLine");
    LLine = hardwareMap.get(ColorSensor.class, "LLine");
    RSkyStoneColor = hardwareMap.get(ColorSensor.class, "RSkyStoneColor");
    LSkyStoneColor = hardwareMap.get(ColorSensor.class, "LSkyStoneColor");
    
    
    //init all of the motors and set their directions
    fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fl.setDirection(DcMotorSimple.Direction.REVERSE);
    bl.setDirection(DcMotorSimple.Direction.REVERSE);
    fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    ((DcMotorEx) fr).setTargetPositionTolerance(50);
    ((DcMotorEx) br).setTargetPositionTolerance(50);
    ((DcMotorEx) fl).setTargetPositionTolerance(50);
    ((DcMotorEx) bl).setTargetPositionTolerance(50);
    
    
    //init servos and their starting positions
    rWaffleGrabber.setDirection(Servo.Direction.REVERSE);
    lWaffleGrabber.setDirection(Servo.Direction.FORWARD);
    RSkystonePivot.setDirection(Servo.Direction.FORWARD);
    LSkystonePivot.setDirection(Servo.Direction.REVERSE);
    RSkystoneGrab.setDirection(Servo.Direction.FORWARD);
    LSkystoneGrab.setDirection(Servo.Direction.REVERSE);
    
    
    rWaffleGrabber.setPosition(0.5);
    lWaffleGrabber.setPosition(0.5);
    
    //closed
    LSkystoneGrab.setPosition(0.5);
    RSkystoneGrab.setPosition(0.5);
    //up
    LSkystonePivot.setPosition(0.54);
    RSkystonePivot.setPosition(0.49);
    
    IMUInit();
    
    
    do
    {
      
      if(gamepad1.dpad_down)
      {//wall
        endPos = 0;
      }
      else if(gamepad1.dpad_up)
      {//bridge
        endPos = 1;
      }
      
      
      if(gamepad1.a && gamepad1.b)
      {//Loading Zone Red
        startPos = 0;
      }
      else if(gamepad1.y && gamepad1.b)
      {//Build Site Red
        startPos = 1;
      }
      else if(gamepad1.a && gamepad1.x)
      {//Loading Zone Blue
        startPos = 2;
      }
      else if(gamepad1.y && gamepad1.x)
      {//Build Site Blue
        startPos = 3;
      }
      
      telemetry.addLine("Select Start & End Position Using Gamepad 1");
      telemetry.addData("Loading Zone Red", "Press A & B");
      telemetry.addData("Build Zone Red", "Press Y & B");
      telemetry.addData("Loading Zone Blue", "Press A & X");
      telemetry.addData("Build Zone Blue", "Press Y & X");
      telemetry.addData("startPos:",startPos);
      //telemetry.addLine("Select End Position Using Gamepad 1");
      telemetry.addData("Wall", "Press Dpad Down");
      telemetry.addData("Neutral SkyBridge", "Dpad Up");
      telemetry.addData("endPos:",endPos);
                telemetry.addData("rightdistance", RDistance.getDistance(DistanceUnit.INCH));

      telemetry.update();
    }
    while ((startPos == -1 || endPos == -1)&& !isStopRequested() && !opModeIsActive());
    
    //display that the robot is initilized & ready to begin the round
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    
    waitForStart();
    
    //telemetry.addData("Angle", getAngle());
    ///*
     //these are me testing to make sure the functions work as intended
     //moveDistance(2,4,0.5);
     //sleep(5000);
     //moveRotations(0, 1);
     //moveRotations(0, -1);
     //moveRotations(1, 1);
     //moveRotations(1, -1);
     //moveRotations(2, 1);
     //moveRotations(2, -1);
     //moveRotations(3, 1);
     //moveRotations(3, -1);
     //moveRotations(4, 1);
     //moveRotations(4, -1);
     //turnIMU(90,0.25);
     //sleep(2000);
     //turnIMU(-90,0.25);
     //moveFoundation(0);
     //moveAcrossLine(2,1);
     //*/
     
    
    //use a switch to determine what tasks to acomplish based off the starting positon
    switch (startPos)
    {
      case 0://loading zone Red
      {
        //move until in scanning range of the quarry
        //moveRotations(1,-1.8);
        moveDistance(2,0,1);
        moveRotations(1,-0.1);
        //scan for the skystone & move to the first Skystone based on the skyStonePos
        scanForSkystone();
        //grab && deposite the skyStone using the grabSkystone() function
        
        grabSkystone();
        
        //turnIMU((getAngle()*-0.01), 0.5);
        //go to the second Skystone based on the distance from the wall to the robot when the first one was picked up
        switch(skyStonePos){
          case 1:
          {
            moveDistance(0,5,1);
            break;
          }
          case 2:
          {
            moveDistance(0,13,1);
            break;
          }
          case 3:
          {
            moveDistance(0,21,1);
            break;
          }
          default:
          {
            moveDistance(0,4,1);
            break;
          }
        }
        //turnIMU((getAngle()*-0.001), 0.5);
         //moveRotations(1,-0.4);
        moveDistance(2,0,0.5);
        grabSkystone();
        
        //park under the SkyBridge acording to alliance partner
        switch(endPos)
        {
          case 0://park closest to the wall
          {
            moveRotations(0,1);
            moveRotations(1,-0.75);
            break;
          }
          case 1://park closest to the bridge
          {
            moveRotations(0,1);
            moveRotations(1,-0.75);
            break;
          }
          default:
          {
            moveRotations(0,1);
            moveRotations(1,-0.75);
            break;
          }
        }
        
      }
      break;
      case 1:// build site Red
      {
        //straif over so alligned with roughly the middle of the foundation
        moveRotations(1,1);
        //go grab the foundation using the touch sensors
        moveFoundation();
        //park the robot under the skybridge
        switch(endPos)
        {
          case 0://park closest to the wall
          {
            moveAcrossLine(0, 1);
            moveRotations(0,-0.5);
            break;
          }
          case 1://park closest to the bridge
          {
            moveRotations(1, -1.5);
            moveAcrossLine(0, 1);
            moveRotations(0,-0.5);
            break;
          }
          default:
          {//if in doubt, park close to the wall
            moveAcrossLine(0, 1);
            moveRotations(0,-0.5);
            break;
          }
        }
        
        break;
      }
      case 2://loading zone Blue
      {
        //move until in scanning range of the quarry
        moveDistance(3,0,1);
        moveRotations(1,0.1);
        //scan for the skystone & move to the first Skystone based on the skyStonePos
        scanForSkystone();
        //grab && deposite the skyStone using the grabSkystone() function
        
        grabSkystone();
        
        //turnIMU((getAngle()*-0.001), 0.5);
        //go to the second Skystone based on the distance from the wall to the robot when the first one was picked up
        switch(skyStonePos){
          case 1:
          {
            moveDistance(0,5,1);
            break;
          }
          case 2:
          {
            moveDistance(0,13,1);
            break;
          }
          case 3:
          {
            moveDistance(0,21,1);
            break;
          }
          default:
          {
            moveDistance(0,4,1);
            break;
          }
        }
        //turnIMU((getAngle()*-0.0001), 0.5);
         //moveRotations(1,0.4);
         moveDistance(3,0,0.5);
        
        grabSkystone();
        
        //park under the SkyBridge acording to alliance partner
        switch(endPos)
        {
          case 0://park closest to the wall
          {
            moveRotations(0,1);
            moveRotations(1,0.7);
            break;
          }
          case 1://park closest to the bridge
          {
            moveRotations(0,1);
            moveRotations(1,0.7);
            break;
          }
          default:
          {
            moveRotations(0,1);
            moveRotations(1,0.7);
            break;
          }
        }
      }
      break;
      case 3:// build site Blue
      {
        //straif over so alligned with roughly the middle of the foundation
        moveRotations(1,-1);
        //go grab the foundation using the touch sensors
        moveFoundation();
        //park under the SkyBridge acording to alliance partner
        switch(endPos)
        {
          case 0://park closest to the wall
          {
            moveAcrossLine(0, 1);
            moveRotations(0,-0.5);
            break;
          }
          case 1://park closest to the bridge
          {
            moveRotations(1, 1);
            moveAcrossLine(0, 1);
            moveRotations(0,-0.5);
            break;
          }
          default:
          {
            moveAcrossLine(0, 1);
            break;
          }
        }
      }
      default:
      {
        break;
      }
    }
    
  }
  
  public void scanForSkystone(){
    
    //make sure the motors are not set to run to position
    fl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    bl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    fr.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    br.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    //run through a loop until the Skystone is seen or stop is pressed
    while (opModeIsActive() && !color() && FDistance.getDistance(DistanceUnit.INCH) > 2)
    {
      fr.setPower(0.5);
      br.setPower(0.5);
      fl.setPower(0.5);
      bl.setPower(0.5);
      telemetry.addData("color", color());
    }
    
    //stop the motors
    fr.setPower(0);
    br.setPower(0);
    fl.setPower(0);
    bl.setPower(0);
    
    // using the robot's distance form the wall determine what position the Skystone is in so that it doesn't have to scan the entire line over again
    //the position is determined by if the robot is in the specific range of a block
    if(FDistance.getDistance(DistanceUnit.INCH) > 40 && FDistance.getDistance(DistanceUnit.INCH) <= 48)
    {//position 3; farthest from the wall
      skyStonePos = 3;
    }
    else if(FDistance.getDistance(DistanceUnit.INCH) > 32 && FDistance.getDistance(DistanceUnit.INCH) <= 40)
    {//position 2; the middle stone
      skyStonePos = 2;
    }
    else if(FDistance.getDistance(DistanceUnit.INCH) > 24 && FDistance.getDistance(DistanceUnit.INCH) <= 32)
    {//position 1; closest to the wall
      skyStonePos = 1;
    }
    else
    {//if in doubt assume it is in position 3
      skyStonePos = 3;
    }
  }
  
  /*
   * determine which color sensor to use based off the starting position 
   * return true if seeing the Skystone & false if otherwise
   */
  public boolean color(){
    float Hue = 0;
    
    switch (startPos){
      case 0:
        {//for the red side use the right sensor
         Hue = JavaUtil.colorToHue(Color.argb(RSkyStoneColor.alpha(), RSkyStoneColor.red(), RSkyStoneColor.green(),RSkyStoneColor.blue()));
          break;
        }
      case 2:
          {//for the blue side use the left sensor
           Hue = JavaUtil.colorToHue(Color.argb(LSkyStoneColor.alpha(), LSkyStoneColor.red(), LSkyStoneColor.green(),LSkyStoneColor.blue()));
            break;
          }
    }
    
    if(Hue > 90)//black
      {
          return true;
      }
      else if((Hue < 90 && Hue >= 40))//yellow
      {
          return false;
      }
      else{
        return false;
      }
    
     
    
  }
  
  /*
   * This function reverses & uses touch sensors to stop & acctuate the foundation grabbers
   * then it drives until about 5cm from the wall & turns based on staring position
   */
  public void moveFoundation(){
    fl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    bl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    fr.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    br.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    
    //set motors to go backwards just slow enough for the foundation not to bounce out of the range of the foundation grabbers
    fr.setPower(-0.3);
    br.setPower(-0.3);
    fl.setPower(-0.3);
    bl.setPower(-0.3);
    
    //set some booleans to triger the end of the loop when the touch sensors are pressed
    boolean isPressedR = false;
    boolean isPressedL = false;
    
    //check to see if the touch sensors have been pressed
    do{
      if(RTouch.isPressed())
      {//if the right side is pressed then grab the foundation & stop moving the right side
        rWaffleGrabber.setPosition(1);
        fr.setPower(0);
        br.setPower(0);
        isPressedR = true;
      }
      
      if(LTouch.isPressed())
      {//if the left side is pressed then grab the foundation & stop moving the left side
        lWaffleGrabber.setPosition(1);
        fl.setPower(0);
        bl.setPower(0);
        isPressedL = true;
      }
      sleep(100);
    }while(opModeIsActive() && !(isPressedR && isPressedL));
    
    //make sure all motor powers are set to zero
    fr.setPower(0);
    br.setPower(0);
    fl.setPower(0);
    bl.setPower(0);
    sleep(300);
    //adgust the robot so that the front is still parallel with the wall
    turnIMU(getAngle(),0.5);
    
    //move the robot & foundation until 10 inches from the wall
    moveDistance(0, 10, 0.5);
    
    //turn the foundation in to the Build Site based on either red or blue alliance
    
    switch(startPos){
      case 1://red
      {
        //moveRotations(1,0.5);
        //moveRotations(4,-1);
        turnIMU(-90,0.75);
        moveRotations(1,0.75);
        break;
      }
      case 3://Blue
      {
        //moveRotations(1,-0.5);
        //moveRotations(4,1);
        turnIMU(90,0.75);
        moveRotations(1,-0.75);
        break;
      }
      default://if in doubt don't turn the foundation
      {
        break;
      }
    }
    
    //release the foundation
    rWaffleGrabber.setPosition(0.5);
    lWaffleGrabber.setPosition(0.5);
  }
  
  /*
   * this functions grabs the Skystone the startPos to determine which side to actuate
   */
  public void grabSkystone()
  {
    switch(startPos){
      case 0:{//RED
      //open the grabber
        RSkystoneGrab.setPosition(0.75);
        sleep(400);
        //rotate arm down
        RSkystonePivot.setPosition(0.1);
        sleep(400);
        //close the grabber
        RSkystoneGrab.setPosition(0.5);
        sleep(400);
        //rotate arm up
        RSkystonePivot.setPosition(0.4);
        sleep(400);
        moveRotations(1, 0.4);
        
        //reverse under the skybridge
        moveAcrossLine(0, -1);
        moveRotations(0, -1.7);
        //rotate down
        RSkystonePivot.setPosition(0.1);
        sleep(400);
        //grabber therefore dropping the SkyStone
        RSkystoneGrab.setPosition(0.75);
        sleep(400);
        //arm back to the up position
        RSkystonePivot.setPosition(0.49);
        sleep(400);
        //return grabber to the closed position
        RSkystoneGrab.setPosition(0.5);
        sleep(400);
        break;
      }
      case 2://Blue
      {
        //open the grabber
        LSkystoneGrab.setPosition(0.75);
        sleep(400);
        //rotate arm down
        LSkystonePivot.setPosition(0.15);
        sleep(400);
        //close the grabber
        LSkystoneGrab.setPosition(0.54);
        sleep(400);
        //rotate arm up
        LSkystonePivot.setPosition(0.45);
        sleep(400);
        moveRotations(1, -0.35);
        turnIMU((getAngle()*0.0001), 0.5);
        //reverse under the skybridge
        moveAcrossLine(0, -1);
        moveRotations(0, -1.7);
        //rotate both arms down
        LSkystonePivot.setPosition(0.15);
        sleep(400);
        //open both grabbers therefore dropping the SkyStone
        LSkystoneGrab.setPosition(0.75);
        sleep(400);
        //rotate both arms back to the up position
        LSkystonePivot.setPosition(0.54);
        sleep(400);
        //return both grabbers to the closed position
        LSkystoneGrab.setPosition(0.5);
        sleep(400);
        break;
      }
      default://if in doubt, do both
      {
        //open both grabbers
        RSkystoneGrab.setPosition(0.75);
        LSkystoneGrab.setPosition(0.75);
        sleep(400);
        //rotate both arms down
        RSkystonePivot.setPosition(0.1);
        LSkystonePivot.setPosition(0.15);
        sleep(400);
        //close both grabbers
        RSkystoneGrab.setPosition(0.5);
        LSkystoneGrab.setPosition(0.5);
        sleep(400);
        //rotate both arms up
        RSkystonePivot.setPosition(0.4);
        LSkystonePivot.setPosition(0.5);
        sleep(400);
        
        //reverse under the skybridge
    moveAcrossLine(0, -1);
    moveRotations(0, -1.7);
    //rotate both arms half way down
    RSkystonePivot.setPosition(0.195);
    LSkystonePivot.setPosition(0.345);
    sleep(400);
    //open both grabbers therefore dropping the SkyStone
    RSkystoneGrab.setPosition(0.75);
    LSkystoneGrab.setPosition(0.75);
    sleep(400);
    //rotate both arms back to the up position
    RSkystonePivot.setPosition(0.49);
    LSkystonePivot.setPosition(0.54);
    sleep(400);
    //return both grabbers to the closed position
    RSkystoneGrab.setPosition(0.5);
    LSkystoneGrab.setPosition(0.5);
    sleep(400);
        break;
      }
    }
    
    
    
  }
  
  /*
   * move the robot until accros the line
   * direction :0 = forward & backward
   *            1 = straif right & left (- == right && + == left)
   *            2 = diagonal to the left
   *            3 = diagonal to the right
   */
  public void moveAcrossLine(int direction, double pwr){
    //set the variables that record the hue of the ground beneath them
    float RHue = JavaUtil.colorToHue(Color.argb(RLine.alpha(), RLine.red(), RLine.green(),RLine.blue()));
    float LHue = JavaUtil.colorToHue(Color.argb(LLine.alpha(), LLine.red(), LLine.green(),LLine.blue()));
    
    fl.setMode(DcMotor.RunMode.RESET_ENCODERS);
    bl.setMode(DcMotor.RunMode.RESET_ENCODERS);
    fr.setMode(DcMotor.RunMode.RESET_ENCODERS);
    br.setMode(DcMotor.RunMode.RESET_ENCODERS);
    fl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    bl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    fr.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    br.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    //use a switch to set the motor powers
    switch(direction){
      case 0://move forward(+) & backward(-)
      {
        fr.setPower(pwr);
        br.setPower(pwr);
        fl.setPower(pwr);
        bl.setPower(pwr);
        break;
      }
      case 1://straif left(+) & right(-)
      {
        fr.setPower(-pwr);
        br.setPower(pwr);
        fl.setPower(pwr);
        bl.setPower(-pwr);
        break;
      }
      case 2://Diagonal with the front right corrner as the forward direction
      {
        fr.setPower(0);
        br.setPower(pwr);
        fl.setPower(pwr);
        bl.setPower(0);
        break;
      }
      case 3://Diagonal with the front left corrner as the forward direction
      {
        fr.setPower(pwr);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(pwr);
        break;
      }
      default:// if in doubt, stop & return
      {
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        return;
      }
    }
    //run the robot until either side crosses the red or blue tape
    do{
      //update the hue values every time the loop is ran
      RHue = JavaUtil.colorToHue(Color.argb(RLine.alpha(), RLine.red(), RLine.green(),RLine.blue()));
      LHue = JavaUtil.colorToHue(Color.argb(LLine.alpha(), LLine.red(), LLine.green(),LLine.blue()));
    }
    while(opModeIsActive() && ((!(RHue > 150 && RHue < 225) && !(RHue > 350 || RHue < 30))&&(!(LHue > 150 && LHue < 225) && !(LHue > 350 || LHue < 30))));
    
    //stop the robot
    fr.setPower(0);
    br.setPower(0);
    fl.setPower(0);
    bl.setPower(0);
    
  }
  
  /*
   * this function makes the robot move forwards or backwards until the imput distance from an object to the front or back of the robot 
   * direction: 0 = forward
   *            1 = backward
   *            2 = right
   *            3 = left
   * distance: measured in INCHES tells how far away you should be from an object
   */
  public void moveDistance(int direction, double distance, double pwr){
    //make sure the input power is positive
    pwr = Math.abs(pwr);
    
    fl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    bl.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    fr.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    br.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    //use a switch to set the motor powers and target positions
    switch(direction){
      case 0://move forward
      {
        //set motor power to forward
        fr.setPower(pwr);
        br.setPower(pwr);
        fl.setPower(pwr);
        bl.setPower(pwr);
        
        //repeat while the front of the robot is farther from an object than the input distance 
        while((double)FDistance.getDistance(DistanceUnit.INCH) > distance && opModeIsActive())
        {
          telemetry.addData("forwardDistance", FDistance.getDistance(DistanceUnit.INCH));
          telemetry.addData("backDistance", FDistance.getDistance(DistanceUnit.INCH));
          telemetry.addData("rightdistance", RDistance.getDistance(DistanceUnit.INCH));
          telemetry.addData("leftdistance", LDistance.getDistance(DistanceUnit.INCH));
          telemetry.update();
        }
        break;
      }
      case 1://move backward
      {
        //set the motor power to reverse
        fr.setPower(-pwr);
        br.setPower(-pwr);
        fl.setPower(-pwr);
        bl.setPower(-pwr);
        
        //repeat while the back of the robot is farther from an object than the input distance 
        while((double)BDistance.getDistance(DistanceUnit.INCH) > distance && opModeIsActive())
        {
          telemetry.addData("forwardDistance", FDistance.getDistance(DistanceUnit.INCH));
          telemetry.addData("backDistance", BDistance.getDistance(DistanceUnit.INCH));
          telemetry.addData("rightdistance", RDistance.getDistance(DistanceUnit.INCH));
          telemetry.addData("leftdistance", LDistance.getDistance(DistanceUnit.INCH));
          telemetry.update();
        }
        break;
      }
        case 2://move right
      {
        //set motor power to strafe right
        fr.setPower(pwr);
        br.setPower(-pwr);
        fl.setPower(-pwr);
        bl.setPower(pwr);
        
        //repeat while the right of the robot is farther from an object than the sensor can read 
        while(Double.isNaN((double)RDistance.getDistance(DistanceUnit.INCH))&& opModeIsActive())
        {
          telemetry.addData("forwardDistance", FDistance.getDistance(DistanceUnit.INCH));
          telemetry.addData("backDistance", FDistance.getDistance(DistanceUnit.INCH));
          telemetry.addData("rightdistance", RDistance.getDistance(DistanceUnit.INCH));
          telemetry.addData("leftdistance", LDistance.getDistance(DistanceUnit.INCH));
          telemetry.update();
          //sleep(20);
          
        }
        break;
      }
      case 3://move left
      {
        //set motor power to strafe left
        fr.setPower(-pwr);
        br.setPower(pwr);
        fl.setPower(pwr);
        bl.setPower(-pwr);
        
        //repeat while the left of the robot is farther from an object than the sensor can read 
        while(Double.isNaN((double)LDistance.getDistance(DistanceUnit.INCH))&& opModeIsActive())
        {
          telemetry.addData("forwardDistance", FDistance.getDistance(DistanceUnit.INCH));
          telemetry.addData("backDistance", FDistance.getDistance(DistanceUnit.INCH));
          telemetry.addData("rightdistance", RDistance.getDistance(DistanceUnit.INCH));
          telemetry.addData("leftdistance", LDistance.getDistance(DistanceUnit.INCH));
          telemetry.update();
          //sleep(50);
          
          
        }
        break;
      }
      default://if in doubt, set motor power to zero & exit the function
      {
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        return;
      }
    }
    //set the motor powers to zero
    fr.setPower(0);
    br.setPower(0);
    fl.setPower(0);
    bl.setPower(0);
  }
  
  /*
   * this function makes the robot move to a position in any direction
   * direction: 0 = forward & backward
   *            1 = straif right & left (+ == right && - == left)
   *            2 = diagonal to the right
   *            3 = diagonal to the left
   *            4 = turn (+ == right - == left)
   * rotations: this tells the motors to move to a position
   */
  public void moveRotations(int direction, double rotations){
    //convert the # of rotations into encoder ticks
    rotations = rotations * 2240;
    //set the motor speed
    double power = 0.75;
    //get the current position of the motors so that the # of rotations can be used without the need to reset the encoders
    int frCurPos = fr.getCurrentPosition();
    int brCurPos = br.getCurrentPosition();
    int flCurPos = fl.getCurrentPosition();
    int blCurPos = bl.getCurrentPosition();
    
    //use a switch to set the motor target positions to the current positions + or - the # of rotations depending upon the input direction
    switch(direction){
      case 0://forward & backward
      {
        fr.setTargetPosition((int)(frCurPos + rotations));
        br.setTargetPosition((int)(brCurPos + rotations));
        fl.setTargetPosition((int)(flCurPos + rotations));
        bl.setTargetPosition((int)(blCurPos + rotations));
        break;
      }
      case 1://straif Right & left
      {
        fr.setTargetPosition((int)(frCurPos - rotations));
        br.setTargetPosition((int)(brCurPos + rotations));
        fl.setTargetPosition((int)(flCurPos + rotations));
        bl.setTargetPosition((int)(blCurPos - rotations));
        break;
      }
      case 2://Diagonal to the front right corner
      {
        fr.setTargetPosition(frCurPos);
        br.setTargetPosition((int)(brCurPos + rotations));
        fl.setTargetPosition((int)(flCurPos + rotations));
        bl.setTargetPosition(blCurPos);
        break;
      }
      case 3://Diagonal to the front left corner
      {
        fr.setTargetPosition((int)(frCurPos - rotations));
        br.setTargetPosition(brCurPos);
        fl.setTargetPosition(flCurPos);
        bl.setTargetPosition((int)(blCurPos - rotations));
        break;
      }
      case 4://turn (+) == right && (-)== left
      {
        fr.setTargetPosition((int)(frCurPos - rotations));
        br.setTargetPosition((int)(brCurPos - rotations));
        fl.setTargetPosition((int)(flCurPos + rotations));
        bl.setTargetPosition((int)(blCurPos + rotations));
        break;
      }
    }
    
    //set the motors to run to position
    fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    //set the motors to the set power level
    fr.setPower(power);
    br.setPower(power);
    fl.setPower(power);
    bl.setPower(power);
    
    //create a boolean that will be used to tell if the motors are still busy
    boolean motors;
    do
    {
      //update the boolean 
      motors = ((fr.isBusy()||br.isBusy())||(fl.isBusy()||bl.isBusy()));
      telemetry.addData("frCurPos", frCurPos);
      telemetry.addData("brCurPos", brCurPos);
      telemetry.addData("flCurPos", flCurPos);
      telemetry.addData("blCurPos", blCurPos);
      telemetry.update();
    }
    while(opModeIsActive() && motors);
    
    //make sure all motors are set to zero power
    fr.setPower(0);
    br.setPower(0);
    fl.setPower(0);
    bl.setPower(0);
  }
  
  /*
   *This function turns the robot based on the expansion hub IMU
   *  degrees = (+) left && (-) right 
   */
  public void turnIMU(double degrees, double power){
    
    fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    //make sure the power being imput is positive
    power = Math.abs(power);
    //set seprate power values for the right & left
    double  leftPower, rightPower;
    
    // restart imu movement tracking.
    resetAngle();
    
    // getAngle() returns + when rotating counter clockwise (right) and - when rotating clockwise (left)
    
    if (degrees > 0)
    {   // turn left
      leftPower = power;
      rightPower = -power;
    }
    else if (degrees < 0)
    {   // turn right
      leftPower = -power;
      rightPower = power;
    }
    else return;
    
    // set power to rotate.
    fl.setPower(leftPower);
    fr.setPower(rightPower);
    bl.setPower(leftPower);//
    br.setPower(rightPower);
    
    // rotate until turn is completed.
    if (degrees < 0){//turn right
      while (opModeIsActive() && (getAngle() >= degrees)) {
        // display motor powervalues
        telemetry.addData("frPwr", fr.getPower());
        telemetry.addData("brPwr", br.getPower());
        telemetry.addData("flPwr", fl.getPower());
        telemetry.addData("blPwr", bl.getPower());
        telemetry.addData("angle", getAngle());
        telemetry.addData("angle of IMU", imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
      }
    }
    else
    {//turn left
      while (opModeIsActive() && (getAngle() <= degrees)) {
        // display motor powervalues
        telemetry.addData("frPwr", fr.getPower());
        telemetry.addData("brPwr", br.getPower());
        telemetry.addData("flPwr", fl.getPower());
        telemetry.addData("blPwr", bl.getPower());
        telemetry.addData("angle", getAngle());
        telemetry.addData("angle of IMU", imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
      }
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
    lastAngles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    
    globalAngle = 0;
  }
  
  /*
   * Get current cumulative angle rotation from last reset.
   * @return Angle in degrees. - = left, + = right.
   */
  private double getAngle()
  {
    /* We  determined the Y axis is the axis we want to use for heading angle.
     * We have to process the angle because the imu works in euler angles so the Y axis is
     * returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
     * 180 degrees. We detect this transition and track the total cumulative angle of rotation.
     */
    
    double angles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    
    double deltaAngle = angles - lastAngles;
    
    //angles += 180;
    if (deltaAngle < -180){
      deltaAngle += 360;
    }
    else if (deltaAngle > 180){
      deltaAngle -= 360;
    }
    
    globalAngle += deltaAngle;
    
    lastAngles = angles;
    
    return globalAngle;
    //return angles;
  }
}

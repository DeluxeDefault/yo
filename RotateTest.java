package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="RotateTest")
//@Disabled
public class RotateTest extends LinearOpMode
{
    DcMotor                 lf, lb, rf, rb;
    TouchSensor             touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
   

private DcMotor slide;
private DcMotor slide2;
private Servo r2; 
private Servo r1;
private Servo f1;
private Servo f2;
private Servo claw;

private int rfPos;
private int rbPos;
private int lfPos;
private int lbPos;
private int slidePos;
private int slide2Pos;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
       rb = hardwareMap.dcMotor.get("rb");
    lb = hardwareMap.dcMotor.get("lb");
    lf = hardwareMap.dcMotor.get("lf");
     rf = hardwareMap.dcMotor.get("rf");
     slide = hardwareMap.dcMotor.get("slide");
     slide2 = hardwareMap.dcMotor.get("slide2");
    r1 = hardwareMap.servo.get("r1");
    r2 = hardwareMap.servo.get("r2");
   f1 = hardwareMap.servo.get("f1");
   f2 = hardwareMap.servo.get("f2");
   claw = hardwareMap.servo.get("claw");

        

          
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        rbPos = 0;
        rfPos = 0;
        lbPos = 0;
        lfPos = 0;
        slidePos = 0;
        slide2Pos = 0;
        // get a reference to touch sensor.
       

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();
         

        telemetry.addData("Mode", "running");
        telemetry.update();

        
            

        // drive until end of period.

        while (opModeIsActive())
        {
            // Use gyro to drive in a straight line.
            correction = checkDirection();

            

            lf.setPower(power - correction);
            rf.setPower(power + correction);
             lb.setPower(power - correction);
            rb.setPower(power + correction);

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

{
               

               
          rotate(-42, .2);
                drive(1200,1200,1200,1200, 0,0, .2);

                
            }
        

        // turn the motors off.
        rf.setPower(0);
        rb.setPower(0);
         lf.setPower(0);
                lb.setPower(0);
    }
}
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
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

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  lbPower, rbPower, lfPower, rfPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            lbPower = power;
            rbPower = -power;
            lfPower = power;
            rfPower = -power;
            telemetry.addData("1", correction);
            telemetry.update();
        }
        else if (degrees > 0)
        {   // turn left.
            lfPower = -power;
            rfPower = power;
            lbPower = -power;
            rbPower = power;
            telemetry.addData("2", correction);
            telemetry.update();
        }
        else return;

        // set power to rotate.
        lf.setPower(lfPower);
        rf.setPower(rfPower);
         lb.setPower(lbPower);
        rb.setPower(rbPower);
        telemetry.addData("3", correction);
            telemetry.update();

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                
                telemetry.addData("4", correction);
            telemetry.update();
            }

            while (opModeIsActive() && getAngle() > degrees) {
                lf.setPower(.3);
                lb.setPower(.3);
                rb.setPower(-.3);
                rf.setPower(-.3);
            telemetry.addData("5", correction);
            telemetry.update();
            }
        }
        else    // left turn.
            if (opModeIsActive() && getAngle() < degrees) {
                lf.setPower(-.3);
                lb.setPower(-.3);
                rb.setPower(.3);
                rf.setPower(.3);
            telemetry.addData("6", correction);
            telemetry.update();
            }

        // turn the motors off.
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        telemetry.addData("7", correction);
            telemetry.update();

        // wait for rotation to stop.
        
sleep(1000);
        // reset angle tracking on new heading.
        resetAngle();
    }
        public void servox(){
            r2.setPosition(-.6);
  
    f1.setPosition(.54);
  f2.setPosition(.24);
  
        
    }
    
    public void closeclaw(){
        claw.setPosition(1);
    }
    
    public void openclaw(){
    claw.setPosition(-1);
    }
    
    public void servoy(){
        r2.setPosition(.5); 
  
  
  f1.setPosition(-1);
  f2.setPosition(.7);
    }
    
    public void mpause (){
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
        sleep(200);
        
    }
     public void spause (){
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
        sleep(700);
     }
      public void fpause (){
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
        sleep(30000);
      }
    
    
    public void drive (int rfTarget, int lfTarget, int rbTarget, int lbTarget, int slideTarget, int slide2Target,  double speed) {        

 rbPos += rbTarget;
 rfPos += rfTarget;
 lbPos += lbTarget;
 lfPos += lfTarget;  
slidePos += slideTarget;
slide2Pos += slide2Target;
 
 
 rb.setTargetPosition(rbPos);
 rf.setTargetPosition(rfPos);
 lb.setTargetPosition(lbPos);
 lf.setTargetPosition(lfPos);
slide.setTargetPosition(slidePos);
slide2.setTargetPosition(slide2Pos);
 
 rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

 
 lf.setPower(speed);
 rf.setPower(speed);
 lb.setPower(speed);
 rb.setPower(speed);
 slide.setPower(speed); 
 slide2.setPower(speed); 
 
 
 rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 
 
 
 while (opModeIsActive()  && rf.isBusy() && lf.isBusy() && rb.isBusy () && lb.isBusy()){
 
 idle();
  

 


 
}
}
}
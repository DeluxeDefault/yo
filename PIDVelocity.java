package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous (name = "PIDVelocity")
public class PIDVelocity extends LinearOpMode {
 

    
 DcMotorEx rb;
 DcMotorEx lb;
 DcMotorEx lf;
 DcMotorEx rf;
 DcMotorEx slide;
 DcMotorEx slide2;
 Servo r2; 
 Servo r1;
 Servo f1;
 Servo f2;
 Servo claw;
 ColorSensor color;
 
 private BNO055IMU imu;

double intergalSum = 0;
double Kp = 0;
double Ki = 0;
double Kd = 0;



ElapsedTime timer = new ElapsedTime();
private double lastError = 0;



    
@Override
public void runOpMode() throws InterruptedException {
    rf = hardwareMap.get(DcMotorEx.class, "rf");
    rb = hardwareMap.get(DcMotorEx.class, "rb");
    lf = hardwareMap.get(DcMotorEx.class, "lf");
     lb = hardwareMap.get(DcMotorEx.class, "lb");
     slide = hardwareMap.get(DcMotorEx.class, "slide");
     slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
    r1 = hardwareMap.servo.get("r1");
    r2 = hardwareMap.servo.get("r2");
   f1 = hardwareMap.servo.get("f1");
   f2 = hardwareMap.servo.get("f2");
   claw = hardwareMap.servo.get("claw");
   color = hardwareMap.get(ColorSensor.class,"color" );
   
   
   lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        
        waitForStart();
        
        double referenceAngle = Math.toRadians(90);
        
        
        while (opModeIsActive()) {
        double power = PIDControl(referenceAngle, imu.getAngularOrientation().firstAngle);
       rf.setPower(-power);
       rb.setPower(-power);
       lf.setPower(power);
       lb.setPower(power);
       
        
       
}
}
public double angleWrap(double radians){
 while (radians > Math.PI){
  radians -= 2 * Math.PI;
 }
 while (radians < - Math.PI){
  radians += 2 * Math.PI;
 }
 return radians;
}
public double PIDControl (double reference, double state){
 double error = angleWrap (reference - state);
 intergalSum += error * timer.seconds();
 double derivative = (error - lastError) / timer.seconds();
 lastError = error;
 
 
 timer.reset();
 
 double output = (error * Kp) + (derivative * Kd) + (intergalSum * Ki);
 return output;
 
}
 
 


}




































       /*
       //forward
       drive(300,300,300,300, 0, .2);{
       }
       
      //backwar*/
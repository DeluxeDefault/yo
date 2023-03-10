package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MatchTeleOp")
public class Yeet2Play extends LinearOpMode {

  private DcMotor rb;
  private DcMotor lb;
  private DcMotor lf;
  private DcMotor rf;
  private DcMotor slide;
  private DcMotor slide2;
  private Servo r2;
  private Servo r1;
  private Servo f1;
  private Servo f2;
  private Servo claw;
  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
   
  @Override
  public void runOpMode() {
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
   
     rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        
        double forward = -gamepad1.left_stick_y;
double strafe = gamepad1.left_stick_x;
double turn = gamepad1.right_stick_x;




{
 if (gamepad1.right_bumper){
     forward /=3;
  strafe /=3;
  turn /=3;
 }
 else {
     forward /=1.45;
  strafe /=1.45;
  turn /=1.45;
 
}
rf.setPower(forward - strafe - turn);
lf.setPower(forward + strafe + turn);
lb.setPower(forward - strafe + turn);
rb.setPower(forward + strafe - turn);





if (gamepad2.y){
  
 r2.setPosition(.6);
  
  
  f1.setPosition(-1);
  f2.setPosition(.7);
  
  
  
}
if (gamepad2.x){
  
  r2.setPosition(-1);
  
    f1.setPosition(.54);
  f2.setPosition(.24); 
  
  
  
 


}
if (gamepad2.left_bumper){
      slide.setPower(-.3);
      slide2.setPower(.3);
      telemetry.addData("green","");
 telemetry.update();
  
    }
    else if (gamepad2.right_bumper){
      slide.setPower(1);
      slide2.setPower(-1);
    }
    else  if (gamepad2.right_trigger > 0){
    slide.setPower(.45);
    slide2.setPower(-.45);
}
    else {
      slide.setPower(0);
       slide2.setPower(0);
    }

  


  
  

if (gamepad2.b){
  claw.setPosition(.48);
  
}
if (gamepad2.a){
  claw.setPosition(1);

  
}

      }  
      }
}

  
}
}

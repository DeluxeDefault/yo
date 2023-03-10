package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "b")
public class YoMama extends LinearOpMode {
  
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
private ColorSensor color;


private int rbPos;
private int rfPos;
private int lbPos;
private int lfPos;
private int slidePos;
private int slide2Pos;
  
@Override
public void runOpMode(){
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
  color = hardwareMap.get(ColorSensor.class,"color" );
 
 
  lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
     
      waitForStart();
      
      /*
      //forward
      drive(300,300,300,300, 0, .2);{
      }
     
     //backwards
      drive(-290,-290,-290,-290, 0, .2);{
      }
     
      //slides up
      drive(0,0,0,0,-1000,1);{
      }
     
      {
       r2.setPosition(.5);


 f1.setPosition(-1);
 f2.setPosition(.7);
 sleep(1000);
      }
     
      */
     
      //forward
     
      
     
      
    
    
      
     
     
      while (opModeIsActive()){
      
telemetry.addData("red", color.red());
 telemetry.addData("blue", color.blue());
  telemetry.addData("green", color.green());
 telemetry.update();

 color.green();
 color.blue();
 color.red();

 drive(300,300,300,300, 0,0, .3);
         mpause();
         drive(-290,-290,-290,-290, 0,0, .3);
         mpause();
         servox();
         spause();
         mpause();
         drive(0,0,0,0, -100,100, .4);
         mpause();
         drive(170,170,170,170, 0,0, .4);
         closeclaw();
         spause();
         servoy();
         spause();
         drive(620,620,620,620, 0,0, .05);
         mpause();

 if (color.blue  ()>3000){
     telemetry.addData("blue color",color.blue());
     telemetry.update();
  drive(1400,1400,1400,1400, 0,0, .3);
  mpause();
     drive(-200,-200,-200,-200, 0,0, .3);
     mpause();
  drive(260,-260,260,-260, 0,0, .3);
  mpause();
  drive(0,0,0,0, 880,-880, .7);
  mpause();
  drive(270,270,270,270, 0,0, .3);
  mpause();
  spause();
  spause();
  drive(0,0,0,0, -150,150, .3);
  openclaw();

  spause();
  drive(-1000,1000,-1000,1000, 0,0, .2);
  mpause();
  drive(0,0,0,0, -600,600, .3);
  mpause();
  servox();
  spause();
  drive(940,940,940,940, 0,0, .2);
  mpause();
  closeclaw();
  spause();
  drive(0,0,0,0, 250,-250, .4);
  mpause();
  drive(-940,-940,-940,-940, 0,0, .2);
  mpause();
  servoy();
  drive(0,0,0,0, 600,-600, .7);
  mpause();
  drive(1030,-1030,1030,-1030, 800,800, .2);
  spause();
  spause();
     drive(0,0,0,0, -300,300, .3);




 


  fpause();
 
 }

  else if (color.green()>3100){
  drive(1200,1200,1200,1200, 0,0, .2);
  drive(-740,740,-740,740, 0,0, .2);
   drive(300,300,300,203, 0,0, .2);
 
 }

 else if (color.red ()>2800){
   drive(1100,1100,1100,1100, 0,0, .2);
  drive(-740,740,-740,740, 0,0, .2);
  drive(-810,-810,-810,-810, 0,0, .2);
 
 }

rf.setPower(0);
 lf.setPower(0);
  rb.setPower(0);
   lb.setPower(0);


     
      }
     
     
     
      
     
    
     
     
     
    
   
     
    
    
     
     
}
  
   public void mpause (){
   rf.setPower(0);
   lf.setPower(0);
   rb.setPower(0);
   lb.setPower(0);
   sleep(300);
   }
  
   public void spause (){
   rf.setPower(0);
   lf.setPower(0);
   rb.setPower(0);
   lb.setPower(0);
   sleep(800);
   }
  
   public void fpause (){
   rf.setPower(0);
   lf.setPower(0);
   rb.setPower(0);
   lb.setPower(0);
   sleep(30000);
   }
  
   public void servoy (){
    r2.setPosition(.5);


 f1.setPosition(-1);
 f2.setPosition(.7);
   }
  
   public void servox (){
     r2.setPosition(-.6);

   f1.setPosition(.54);
 f2.setPosition(.24);
   }
  
   public void openclaw (){
    claw.setPosition(-1);
   }
  
   public void closeclaw(){
    claw.setPosition(1);
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



while (opModeIsActive()  && rf.isBusy() && lf.isBusy()){

idle();






}


}






                   











  
} 


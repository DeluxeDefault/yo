package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "Auto")
public class Auto extends LinearOpMode {
    public static final String TFOD_MODEL_ASSET = "model_unquant";

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
        

       

       
        
       
        
      
      
        
       
       
       while (opModeIsActive()){
        
 telemetry.addData("red", color.red());
  telemetry.addData("blue", color.blue());
   telemetry.addData("green", color.green());
  telemetry.update();
  
  color.green();
  color.blue();
  color.red();

           closeclaw();
           drive(200,200,200,200, 0,0, .3);
           mpause();
           drive(250,-250,-250,250, 0,0, .3);
           mpause();
           drive(445,445,445,445, 0,0, .3);
           mpause();
           drive(190,190,190,190, 0,0, .05);
  /*
           drive(300,300,300,300, 0,0, .2);
           mpause();
           drive(-290,-290,-290,-290, 0,0, .2);
           mpause();
           openclaw();
           servox();
           spause();
           spause();
           drive(85,85,85,85, 0,0, .2);
          closeclaw();
          spause();
          spause();
          servoy();
          mpause();
          drive(635,635,635,635, 0,0, .15);
          mpause();
          drive(190,190,190,190, 0,0, .05);
*/
  
  if (color.blue  ()>3000){
      telemetry.addData("blue color",color.blue());
      telemetry.update();
  drive(1400,1400,1400,1400, 0,0, .3);
      mpause();
      drive(-200,-200,-200,-200, 0,0, .3);
      mpause();
      servoy();
      drive(0,0,0,0, 1200,-1200, .65); //up
      mpause();
      drive(-315,315,-315,315, 0,0, .25);
      mpause();
      drive(85,85,85,85, 0,0, .25);
      spause();
      drive(0,0,0,0, -550,550, .4); //down
      spause();
      mpause();
      openclaw();
      spause();
      drive(0,0,0,0, 550,-550, .65);
      mpause();
      
      drive(1048,-1048,1048,-1048, 0,0, .25);
      servox();
      drive(0,0,0,0, -1000,1000, .4); //down
      mpause();
      spause();
      drive(-150,150,150,-150, 0,0, .3);
      mpause();
      drive(880,880,880,880, 0,0, .3);
      mpause();
      closeclaw();
      spause();
      spause();
      drive(0,0,0,0, 500,-500, .41); //up\
      mpause();
      drive(-800,-800,-800,-800, 0,0, .3);
      mpause();
      drive(150,-150,-150,150, 0,0, .3);
      mpause();
      servoy();
      drive(0,0,0,0, 500,-500, .65); //up\
      drive(-1135,1135,-1135,1135, 0,0, .25);
      mpause();
      drive(65,65,65,65, 0,0, .3);
      spause();
      drive(0,0,0,0, -550,550, .4);
      spause();
      openclaw();
      spause();
   drive(0,0,0,0, -550,550, .37);
   spause();
   openclaw();
   mpause();
   drive(0,0,0,0, 500,-500, .65); //up\
   drive(-356,356,-356,356, 1000,-1000, .3);
   mpause();
   drive(900,900,900,900, 0,0, .3);
   mpause();
   drive(-660,660,-660,660, 0,0, .3);
   mpause();
   drive(600,600,600,600, 0,0, .3);




   /*
   drive(-1015,1015,-1015,1015, -575,575, .2);
   mpause();
   drive(800,800,800,800, 0,0, .25);
   mpause();
   drive(-500,500,-500,500, 0,0, .25);
   mpause();
   drive(400,400,400,400, 0,0, .25);
   mpause();
   drive(-250,250,-250,250, 0,0, .25);
   mpause();
   drive(400,400,400,400, 0,0, .25);
   mpause();
*/





   

 
   fpause();
   
  }
  
   else if (color.green()>3100){
      drive(1400,1400,1400,1400, 0,0, .3);
      mpause();
      drive(-200,-200,-200,-200, 0,0, .3);
      mpause();
      servoy();
      drive(0,0,0,0, 1200,-1200, .65); //up
      mpause();
      drive(-315,315,-315,315, 0,0, .25);
      mpause();
      drive(85,85,85,85, 0,0, .25);
      spause();
      drive(0,0,0,0, -550,550, .4); //down
      spause();
      mpause();
      openclaw();
      spause();
      drive(0,0,0,0, 550,-550, .65);
      mpause();
      
      drive(1048,-1048,1048,-1048, 0,0, .25);
      servox();
      drive(0,0,0,0, -1000,1000, .4); //down
      mpause();
      spause();
      drive(-150,150,150,-150, 0,0, .3);
      mpause();
      drive(880,880,880,880, 0,0, .3);
      mpause();
      closeclaw();
      spause();
      spause();
      drive(0,0,0,0, 500,-500, .41); //up\
      mpause();
      drive(-800,-800,-800,-800, 0,0, .3);
      mpause();
      drive(150,-150,-150,150, 0,0, .3);
      mpause();
      servoy();
      drive(0,0,0,0, 500,-500, .65); //up\
      drive(-1135,1135,-1135,1135, 0,0, .25);
      mpause();
      drive(65,65,65,65, 0,0, .3);
      spause();
      drive(0,0,0,0, -550,550, .4);
      spause();
      openclaw();
      spause();
      drive(0,0,0,0, 500,-500, .65); //up\
      drive(-1070,1070,-1070,1070, 1000,-1000, .3);
      mpause();
      drive(800,800,800,800, 0,0, .3);
      mpause();


   
  }
  
  else if (color.red ()>2800){
      drive(1400,1400,1400,1400, 0,0, .3);
      mpause();
      drive(-200,-200,-200,-200, 0,0, .3);
      mpause();
      servoy();
      drive(0,0,0,0, 1200,-1200, .65); //up
      mpause();
      drive(-315,315,-315,315, 0,0, .25);
      mpause();
      drive(85,85,85,85, 0,0, .25);
      spause();
      drive(0,0,0,0, -550,550, .4); //down
      spause();
      mpause();
      openclaw();
      spause();
      drive(0,0,0,0, 550,-550, .65);
      mpause();
      
      drive(1048,-1048,1048,-1048, 0,0, .25);
      servox();
      drive(0,0,0,0, -1000,1000, .4); //down
      mpause();
      spause();
      drive(-150,150,150,-150, 0,0, .3);
      mpause();
      drive(880,880,880,880, 0,0, .3);
      mpause();
      closeclaw();
      spause();
      spause();
      drive(0,0,0,0, 500,-500, .41); //up\
      mpause();
      drive(-800,-800,-800,-800, 0,0, .3);
      mpause();
      drive(150,-150,-150,150, 0,0, .3);
      mpause();
      servoy();
      drive(0,0,0,0, 500,-500, .65); //up\
      drive(-1135,1135,-1135,1135, 0,0, .25);
      mpause();
      drive(65,65,65,65, 0,0, .3);
      spause();
      drive(0,0,0,0, -550,550, .4);
      spause();
      openclaw();
      spause();
      
   
   
      drive(0,0,0,0, 550,-550, .65);
      mpause();
      
      drive(-326,326,-326,326, 1000,-1000, .3);
      servox();
      drive(0,0,0,0, -1000,1000, .4); //down
      mpause();
      drive(-800,-800,-800,-800, 0,0, .3);
      mpause();
      drive(692,-692,692,-692, 0,0, .3);
      mpause();
      drive(-750,-750,-750,-750, 0,0, .3);
      mpause();
  }

 rf.setPower(0);
  lf.setPower(0);
   rb.setPower(0);
    lb.setPower(0);
    fpause();
 
  
       
       }
       
       
       
        
       
      
       
       
       
      
     
       
      
      
       
       
}
    
    public void mpause (){
    rf.setPower(0);
    lf.setPower(0);
    rb.setPower(0);
    lb.setPower(0);
    sleep(110);
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
     claw.setPosition(.35);
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

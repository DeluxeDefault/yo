/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode {
    private DcMotor rb;
    private DcMotor lb;
    private DcMotor lf;
    private DcMotor rf;
    private DcMotor slide = null;
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

    OpenCvCamera camera;
    org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag Ids of custom sleeve
    int Left = 0;
    int Middle = 1;
    int Right = 2;


    AprilTagDetection tagOfInterest = null;

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
        color = hardwareMap.get(ColorSensor.class, "color");


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


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == Left || tag.id == Middle || tag.id == Right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }

        /* Actually do something useful */
        else if (tagOfInterest.id == Left) {
            drive(1400,1400,1400,1400, 0,0, .3);
            mpause();
            drive(-200,-200,-200,-200, 0,0, .3);
            mpause();
            servoy();
            drive(0,0,0,0, 1200,-1200, .65); //up
            mpause();
            drive(-382,382,-382,382, 0,0, 3);
            mpause();
            drive(85,85,85,85, 0,0, .25);
            spause();
            drive(0,0,0,0, -550,550, .4); //down
            spause();
            mpause();
            openclaw();
            mpause();
            drive(0,0,0,0, 550,-550, .65);
            mpause();
            drive(1103,-1103,1103,-1103, 0,0, .25);
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
            drive(0,0,0,0, 500,-500, .65); //up\
            drive(-1070,1070,-1070,1070, 1000,-1000, .3);
            mpause();
            drive(800,800,800,800, 0,0, .3);
            mpause();
        } else if (tagOfInterest == null || tagOfInterest.id == Middle) {
            drive(1400,1400,1400,1400, 0,0, .3);
            mpause();
            drive(-200,-200,-200,-200, 0,0, .3);
            mpause();
            servoy();
            drive(0,0,0,0, 1200,-1200, .65); //up
            mpause();
            drive(-382,382,-382,382, 0,0, 3);
            mpause();
            drive(85,85,85,85, 0,0, .25);
            spause();
            drive(0,0,0,0, -550,550, .4); //down
            spause();
            mpause();
            openclaw();
            mpause();
            drive(0,0,0,0, 550,-550, .65);
            mpause();
            drive(1103,-1103,1103,-1103, 0,0, .25);
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
            drive(0,0,0,0, 500,-500, .65); //up\
            drive(-1070,1070,-1070,1070, 1000,-1000, .3);
            mpause();
            drive(800,800,800,800, 0,0, .3);
            mpause();
        } else if (tagOfInterest.id == Right) {
            drive(1400,1400,1400,1400, 0,0, .3);
            mpause();
            drive(-200,-200,-200,-200, 0,0, .3);
            mpause();
            servoy();
            drive(0,0,0,0, 1200,-1200, .65); //up
            mpause();
            drive(-382,382,-382,382, 0,0, 3);
            mpause();
            drive(85,85,85,85, 0,0, .25);
            spause();
            drive(0,0,0,0, -550,550, .4); //down
            spause();
            mpause();
            openclaw();
            mpause();
            drive(0,0,0,0, 550,-550, .65);
            mpause();
            drive(1103,-1103,1103,-1103, 0,0, .25);
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
            drive(0,0,0,0, 500,-500, .65); //up\
            drive(-1070,1070,-1070,1070, 1000,-1000, .3);
            mpause();
            drive(800,800,800,800, 0,0, .3);
            mpause();
        }
        else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }






        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            sleep(20);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void mpause() {
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        sleep(75);
    }

    public void spause() {
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        sleep(800);
    }

    public void fpause() {
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        sleep(30000);
    }

    public void servoy() {
        r2.setPosition(.5);


        f1.setPosition(-1);
        f2.setPosition(.7);
    }

    public void servox() {
        r2.setPosition(-.6);

        f1.setPosition(.54);
        f2.setPosition(.24);
    }

    public void openclaw() {
        claw.setPosition(.35);
    }

    public void closeclaw() {
        claw.setPosition(1);
    }

    public void drive(int rfTarget, int lfTarget, int rbTarget, int lbTarget, int slideTarget, int slide2Target, double speed) {
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


        while (opModeIsActive() && rf.isBusy() && lf.isBusy()) {

            idle();


        }
    }
}
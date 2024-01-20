package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="telopMeet3", group = "a")
@Disabled
public class telopMeet3 extends LinearOpMode {
    public static DcMotor left;
    public static DcMotor right;
    public static DcMotor front;
    public static DcMotor back;
    public static DcMotor inModer;
    public static DcMotor outModer;
    public static DcMotor grabModer;
    public static DcMotor upModer;

    public Servo shooter;

    private DistanceSensor sensorDistanceL;
    private DistanceSensor sensorDistanceR;

    double angleOfset = 0;
    Pose2d myPose;

    boolean blueSide = true;

    double myPose_Heading = 0;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        if(PoseStorage.redSide){
            blueSide = false;
            angleOfset = Math.toRadians(-135);
        } else {
            angleOfset = Math.toRadians(45);
            blueSide = true;
        }
        int grabUpPos = 25060;
        int grabHangPos = 5744;
        int grabCamPos = 3395;
        int grabDownPos = 0;

        int outLoadPos = 0;
        int outDropPos = -580;
        int outLineupPos = -250;

        boolean BIsPressed = false;
        boolean XIsPressed = false;
        boolean YIsPressed = false;

        double driveSpeed = 1;
        left = hardwareMap.get(DcMotor.class, "right");
        right = hardwareMap.get(DcMotor.class, "left");
        front = hardwareMap.get(DcMotor.class, "front");
        back = hardwareMap.get(DcMotor.class, "back");

        inModer = hardwareMap.get(DcMotor.class, "in");
        outModer = hardwareMap.get(DcMotor.class, "out");
        grabModer = hardwareMap.get(DcMotor.class, "grab");
        upModer = hardwareMap.get(DcMotor.class,"up");

        inModer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upModer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outModer.setTargetPosition(outLoadPos);
        outModer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        grabModer.setTargetPosition(grabDownPos);
        grabModer.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        shooter = hardwareMap.get(Servo.class, "shooter");
        //initializing all the motors.



        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        front.setDirection(DcMotorSimple.Direction.FORWARD);
        back.setDirection(DcMotorSimple.Direction.REVERSE);
        inModer.setDirection(DcMotorSimple.Direction.FORWARD);
        outModer.setDirection(DcMotorSimple.Direction.FORWARD);
        grabModer.setDirection(DcMotorSimple.Direction.FORWARD);
        upModer.setDirection(DcMotorSimple.Direction.FORWARD);
        //setting the direction of the motors.

        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "leftdis");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "rightdis");

        Rev2mDistanceSensor sensorTimeOfFlightL = (Rev2mDistanceSensor) sensorDistanceL;
        Rev2mDistanceSensor sensorTimeOfFlightR = (Rev2mDistanceSensor) sensorDistanceR;

        double ldis = 0;
        double rdis = 0;

        double leftPow = 0;
        double rightPow = 0;

        double Ltarget = 1.1;

        double Rtarget = 1.5;

        double PMultiplyer = .4;

        double maxPow = .4;
        double powerSlower = 0;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);

        boolean bDown = false;
        boolean xDown = false;
        boolean yDown = false;

        double gamepad2_right_stick_y = 0;

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            ((DcMotorEx) grabModer).setVelocity(10000);
            outModer.setPower(.4);
            while (opModeIsActive()) {
                drive.update();
                myPose = drive.getPoseEstimate();

                ldis = sensorDistanceL.getDistance(DistanceUnit.INCH);
                rdis = sensorDistanceR.getDistance(DistanceUnit.INCH);

                if (gamepad1.b){
                    bDown = true;
                } else {
                    bDown = false;
                }

                if (gamepad1.x){
                    xDown = true;
                } else {
                    xDown = false;
                }

                if (gamepad1.y){
                    yDown = true;
                } else {
                    yDown = false;
                }

                gamepad2_right_stick_y = gamepad2.right_stick_y;

                myPose_Heading = myPose.getHeading();

                if (bDown && !BIsPressed){
                    BIsPressed = true;
                    if (inModer.getPower()==0){
                        inModer.setPower(-1);
                    } else {
                        inModer.setPower(0);
                    }
                } else if (!bDown) {
                    BIsPressed = false;
                }

                if (xDown && !XIsPressed){
                    XIsPressed = true;
                    if (driveSpeed==1){
                        driveSpeed = .4;
                    } else {
                        driveSpeed = 1;
                    }
                } else if (!xDown) {
                    XIsPressed = false;
                }

                if (gamepad1.a) {
                    inModer.setPower(1);
                }

                if (yDown && !YIsPressed){
                    YIsPressed = true;
                    if (outModer.getTargetPosition()==outLoadPos){
                        outModer.setTargetPosition(outDropPos);
                    } else {
                        outModer.setTargetPosition(outLoadPos);
                    }
                } else if (!yDown) {
                    YIsPressed = false;
                }

                if (gamepad2.dpad_up){
                    grabModer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    grabModer.setTargetPosition(grabUpPos);
                    grabModer.setPower(1);
                } else if (gamepad2.dpad_down) {
                    grabModer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    grabModer.setTargetPosition(grabDownPos);
                    grabModer.setPower(1);
                } else if (gamepad2.left_bumper) {
                    grabModer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    grabModer.setPower(.999);
                } else if (gamepad2.right_bumper && grabModer.getCurrentPosition() > grabDownPos + 800) {
                    grabModer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    grabModer.setPower(-1);
                } else if (grabModer.getPower() < 1){
                    grabModer.setPower(0);
                }

                if (gamepad1.back){
                    shooter.setPosition(.15);
                } else {
                    shooter.setPosition(.05);
                }
                if (gamepad2_right_stick_y < 0 || gamepad2_right_stick_y > 0){
                    upModer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    upModer.setPower(-gamepad2_right_stick_y*.2);
                } else if (gamepad2.b || gamepad1.right_bumper) {
                    upModer.setTargetPosition(0);
                    upModer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    upModer.setPower(.5);
                } else if (gamepad2.y || gamepad1.left_bumper) {
                    upModer.setTargetPosition(665);
                    upModer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    upModer.setPower(.5);
                } else if (upModer.getPower() < 1) {
                    upModer.setPower(0);
                }
                //Setting the shooter power when the right bumper is pressed.
                if (gamepad1.start){
                    angleOfset = 0-myPose_Heading;
                }
                //setting the angle of the robot when start is pressed.
                if(gamepad1.right_stick_button &&
                        ldis < 20 &&
                        Math.abs(ldis - rdis) < 5)
                {

                    leftPow = (ldis - Ltarget) * PMultiplyer;
                    rightPow = (rdis - Rtarget) * PMultiplyer;


                    /*if((ldis + ldis)/2 - (Ltarget+Rtarget)/2 < 10){
                        maxPow = .25;
                    } else if((ldis + ldis)/2 - (Ltarget+Rtarget)/2 < 20){
                        maxPow = .4;
                    } else {
                        maxPow = .6;
                    }*/
                    if(leftPow > maxPow || rightPow > maxPow){
                        if (leftPow > rightPow){
                            powerSlower =  leftPow / maxPow;
                        } else {
                            powerSlower = rightPow / maxPow;
                        }
                        leftPow /= powerSlower;
                        rightPow /= powerSlower;
                    }

                    //move2(-gamepad1.left_stick_x, -leftPow, -rightPow, (ldis - rdis) * .3,.3);
                    if (blueSide){
                        move(-(leftPow+rightPow)/2, gamepad1.left_stick_y, -((ldis-Ltarget) - (rdis-Rtarget)) * .3, 1);

                    }else {
                        move((leftPow+rightPow)/2, gamepad1.left_stick_y, ((ldis-Ltarget) - (rdis-Rtarget)) * .3, 1);
                    }
                } else {
                    move(gamepad1.left_stick_x, gamepad1.left_stick_y, (gamepad1.right_trigger - gamepad1.left_trigger), driveSpeed);
                }
                //moves the robot depending on which direction the left stick is moved.

                /*telemetry.addData("X", drive.getFieldX());
                telemetry.addData("Y", drive.getFieldY());
                telemetry.addData("A", drive.getFieldA());
                telemetry.addData("", "");
                //telemetry.addData("Zone", propDetector.getTargetZone(0));
                telemetry.addData("", "");
                telemetry.addData("RobotXErr", drive.getRobotXErr());
                telemetry.addData("RobotYErr", drive.getRobotYErr());
                telemetry.addData("AErr", drive.getAErr());
                telemetry.addData("DirectToXAngle", drive.getDirectToXAngle());
                telemetry.addData("getFieldXErr", drive.getFieldXErr());
                telemetry.addData("getFieldYErr", drive.getFieldYErr());
                telemetry.addData("", "");
                telemetry.addData("VX", drive.getVx());
                telemetry.addData("VY", drive.getVy());
                telemetry.addData("VA", drive.getVa());
                telemetry.addData("IXer", drive.getIXer());
                telemetry.addData("IDer", drive.getDXer());
                telemetry.addData("armPos", arm.getCurrentPosition());*/
                //telemetry.addData("Lrange", String.format("%.01f in", ldis));
                //telemetry.addData("Rrange", String.format("%.01f in", rdis));;
                //telemetry.addData("angle", Math.toDegrees(myPose_Heading));
                //telemetry.addData("arm pos", outModer.getCurrentPosition());
                //telemetry.addData("grab pos", grabModer.getCurrentPosition());
                //telemetry.addData("angleOfset", angleOfset);
                //telemetry.update();
            }
        }
    }
    public void move(double x, double y, double a, double speed) {
        double rx=0;
        double ry=0;
        rx = -(-(x*Math.cos(myPose_Heading+angleOfset)) + (y*Math.sin(myPose_Heading+angleOfset)));
        ry = (y*Math.cos(myPose_Heading+angleOfset)) + (x*Math.sin(myPose_Heading+angleOfset));
        a*=.8;
        left.setPower((-ry - a) * speed);
        right.setPower((-ry + a) * speed);
        front.setPower((rx + a) * speed);
        back.setPower((rx - a) * speed);

        //left.setPower((-y - a) * speed);
        //right.setPower((-y + a) * speed);
        //front.setPower((x + a) * speed);
        //back.setPower((x - a) * speed);
    }
    public void move2(double x, double y, double y2,double a, double speed) {
        double rx=x;
        double ry=y;
        //rx = -(-(x*Math.cos(myPose.getHeading()+angleOfset)) + (y*Math.sin(myPose.getHeading()+angleOfset)));
        //ry = (y*Math.cos(myPose.getHeading()+angleOfset)) + (x*Math.sin(myPose.getHeading()+angleOfset));
        left.setPower((y));
        right.setPower((y2));
        front.setPower((rx) * speed + a);
        back.setPower((rx) * speed - a);
    }
//Setting the power of the of the motor for the wheels.
}

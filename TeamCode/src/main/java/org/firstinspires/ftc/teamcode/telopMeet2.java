package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp

public class telopMeet2 extends LinearOpMode {
    public static DcMotor left;
    public static DcMotor right;
    public static DcMotor front;
    public static DcMotor back;
    public static DcMotor inModer;
    public static DcMotor outModer;
    public static DcMotor grabModer;
    public static DcMotor upModer;

    public Servo shooter;

    double angleOfset = 0;
    Pose2d myPose;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int grabUpPos = 26594;
        int grabHangPos = 5744;
        int grabCamPos = 3395;
        int grabDownPos = 0;

        int outLoadPos = 0;
        int outDropPos = -600;
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            ((DcMotorEx) grabModer).setVelocity(10000);
            outModer.setPower(.4);
            while (opModeIsActive()) {
                drive.update();
                myPose = drive.getPoseEstimate();

                if (gamepad1.b && !BIsPressed){
                    BIsPressed = true;
                    if (inModer.getPower()==0){
                        inModer.setPower(-1);
                    } else {
                        inModer.setPower(0);
                    }
                } else if (!gamepad1.b) {
                    BIsPressed = false;
                }

                if (gamepad1.x && !XIsPressed){
                    XIsPressed = true;
                    if (driveSpeed==1){
                        driveSpeed = .4;
                    } else {
                        driveSpeed = 1;
                    }
                } else if (!gamepad1.x) {
                    XIsPressed = false;
                }

                if (gamepad1.a) {
                    inModer.setPower(1);
                }

                if (gamepad1.y && !YIsPressed){
                    YIsPressed = true;
                    if (outModer.getTargetPosition()==outLoadPos){
                        outModer.setTargetPosition(outDropPos);
                    } else {
                        outModer.setTargetPosition(outLoadPos);
                    }
                } else if (!gamepad1.y) {
                    YIsPressed = false;
                }

                if (gamepad1.dpad_up){
                    grabModer.setTargetPosition(grabUpPos);
                } else if (gamepad1.dpad_down) {
                    grabModer.setTargetPosition(grabHangPos);
                } else if (gamepad1.dpad_left) {
                    grabModer.setTargetPosition(grabDownPos);
                } else if (gamepad1.dpad_right) {
                    grabModer.setTargetPosition(grabCamPos);
                }

                if (gamepad1.back){
                    shooter.setPosition(.25);
                } else {
                    shooter.setPosition(.75);
                }
                if (gamepad1.right_bumper){
                    upModer.setPower(1);
                } else if (gamepad1.left_bumper) {
                    upModer.setPower(-1);
                } else {
                    upModer.setPower(0);
                }
                //Setting the shooter power when the right bumper is pressed.
                if (gamepad1.start){
                    angleOfset = 0-myPose.getHeading();
                }
                //setting the angle of the robot when start is pressed.
                move(gamepad1.left_stick_x, gamepad1.left_stick_y, (gamepad1.right_trigger - gamepad1.left_trigger), driveSpeed);
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
                telemetry.addData("angle", Math.toDegrees(myPose.getHeading()));
                telemetry.addData("arm pos", outModer.getCurrentPosition());
                telemetry.addData("grab pos", grabModer.getCurrentPosition());
                telemetry.update();
            }
        }
    }
    public void move(double x, double y, double a, double speed) {
        double rx=0;
        double ry=0;
        rx = -(-(x*Math.cos(myPose.getHeading()+angleOfset)) + (y*Math.sin(myPose.getHeading()+angleOfset)));
        ry = (y*Math.cos(myPose.getHeading()+angleOfset)) + (x*Math.sin(myPose.getHeading()+angleOfset));
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
//Setting the power of the of the motor for the wheels.
}

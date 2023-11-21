package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp

public class CrossDriveCam extends LinearOpMode {
    public static DcMotor left;
    public static DcMotor right;
    public static DcMotor front;
    public static DcMotor back;
    public static DcMotor arm;

    public CRServo pixel1;
    public CRServo pixel2;
    public Servo shooter;

    double angleOfset = 0;
    Pose2d myPose;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "right");
        right = hardwareMap.get(DcMotor.class, "left");
        front = hardwareMap.get(DcMotor.class, "front");
        back = hardwareMap.get(DcMotor.class, "back");

        arm = hardwareMap.get(DcMotor.class, "arm");

        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pixel1 = hardwareMap.get(CRServo.class, "pixel1");
        pixel2 = hardwareMap.get(CRServo.class, "pixel2");
        shooter = hardwareMap.get(Servo.class, "shooter");



        // Put initialization blocks here.
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        front.setDirection(DcMotorSimple.Direction.FORWARD);
        back.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        pixel1.setDirection(CRServo.Direction.FORWARD);
        pixel2.setDirection(CRServo.Direction.FORWARD);

       // Drive drive = new Drive();
        PropDetector propDetector = new PropDetector(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                drive.update();
                myPose = drive.getPoseEstimate();
                //if (gamepad1.a){
                //    move(drive.getVx(), -drive.getVy(), drive.getVa(), 0.3);
                //} else {
                if ((-gamepad1.right_stick_y) >= 0 || arm.getCurrentPosition() > 0) {
                    arm.setPower(-gamepad1.right_stick_y);
                } else {
                    arm.setPower(0);
                }
                if (gamepad1.a){
                    pixel1.setPower(.5);
                } else if (gamepad1.y) {
                    pixel1.setPower(-.5);
                }else {
                    pixel1.setPower(0);
                }
                if (gamepad1.x){
                    pixel2.setPower(.5);
                }else if (gamepad1.b){
                    pixel2.setPower(-.5);
                }else {
                    pixel2.setPower(0);
                }
                if (gamepad1.right_bumper){
                    shooter.setPosition(.5);
                } else {
                    shooter.setPosition(0);
                }
                if (gamepad1.start){
                    angleOfset = 0-myPose.getHeading();
                }
                move(gamepad1.left_stick_x, gamepad1.left_stick_y, (gamepad1.right_trigger - gamepad1.left_trigger), 1.5);
                //}
                /*telemetry.addData("X", drive.getFieldX());
                telemetry.addData("Y", drive.getFieldY());
                telemetry.addData("A", drive.getFieldA());
                telemetry.addData("", "");

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
                telemetry.addData("Zone", propDetector.getTargetZone(0));
                telemetry.addData("Zone", propDetector.getData());
                telemetry.addData("arm pos", arm.getCurrentPosition());
                telemetry.addData("angle offset",  angleOfset);
                telemetry.addData("angle", myPose.getHeading());
                telemetry.update();
            }
        }
    }
    public void move(double x, double y, double a, double speed) {
        double rx=0;
        double ry=0;
        rx = -(-(x*Math.cos(myPose.getHeading()+angleOfset)) + (y*Math.sin(myPose.getHeading()+angleOfset)));
        ry = (y*Math.cos(myPose.getHeading()+angleOfset)) + (x*Math.sin(myPose.getHeading()+angleOfset));
        left.setPower((-ry - a) * speed);
        right.setPower((-ry + a) * speed);
        front.setPower((rx + a) * speed);
        back.setPower((rx - a) * speed);
    }

}

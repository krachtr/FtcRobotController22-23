package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="this one", group = "a")

public class telopState extends LinearOpMode {
    double lineUpAve = 0;
    int lineUpAveSize = 1;

    public static DcMotor left;
    public static DcMotor right;
    public static DcMotor front;
    public static DcMotor back;

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
        int outDropPos = 580;
        int outLineupPos = 250;

        boolean BIsPressed = false;
        boolean XIsPressed = false;
        boolean YIsPressed = false;

        double driveSpeed = 1;
        double lineUpA = 0;
        left = hardwareMap.get(DcMotor.class, "right");
        right = hardwareMap.get(DcMotor.class, "left");
        front = hardwareMap.get(DcMotor.class, "front");
        back = hardwareMap.get(DcMotor.class, "back");



        //initializing all the motors.



        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        front.setDirection(DcMotorSimple.Direction.FORWARD);
        back.setDirection(DcMotorSimple.Direction.REVERSE);


        double ldis = 0;
        double rdis = 0;

        double lineUpPow = 0;
        double rightPow = 0;

        double Ltarget = 2.8;

        double Rtarget = 1.4;

        double PMultiplyer = .35;

        double maxPow = 1;
        double powerSlower = 0;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);

        boolean bDown = false;
        boolean xDown = false;
        boolean yDown = false;

        double gamepad2_right_stick_y = 0;

        double realHeading = 0;

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                drive.update();
                myPose = drive.getPoseEstimate();

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

                if (gamepad2.start){
                    blueSide = false;
                } else if (gamepad2.back) {
                    blueSide = true;
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
                    lineUpAve = (lineUpAve * (lineUpAveSize-1) + ((ldis - Ltarget)+(rdis - Rtarget)/2)) / lineUpAveSize;
                    lineUpPow = lineUpAve * PMultiplyer;
                    lineUpA *= .06;

                    if(lineUpPow > maxPow){
                        powerSlower =  lineUpPow / maxPow;
                        lineUpPow /= powerSlower;
                    }

                    //move2(-gamepad1.left_stick_x, -lineUpPow, -rightPow, (ldis - rdis) * .3,.3);
                    realHeading = Math.toDegrees(myPose_Heading+angleOfset);
                    if (realHeading > 180){
                        realHeading -= 360;
                    } else if (realHeading < -180) {
                        realHeading += 360;
                    }

                    if (blueSide){
                        lineUpA = 90 + realHeading;
                        move(-(lineUpPow)/2, gamepad1.left_stick_y * .4, lineUpA * .05, 1);

                    }else {
                        lineUpA = 90 - realHeading;
                        move((lineUpPow)/2, gamepad1.left_stick_y * .4, -lineUpA * .05, 1);
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
                //telemetry.addData("heading", myPose_Heading);
                //telemetry.addData("new heading to deg", Math.toDegrees(myPose_Heading+angleOfset));
                //telemetry.update();
            }
        }
    }
    public void move(double x, double y, double a, double speed) {
        double rx=0;
        double ry=0;
        rx = -(-(x*Math.cos(myPose_Heading+angleOfset)) + (y*Math.sin(myPose_Heading+angleOfset)));
        ry = (y*Math.cos(myPose_Heading+angleOfset)) + (x*Math.sin(myPose_Heading+angleOfset));
        telemetry.addData("rx", rx);
        telemetry.addData("ry", ry);
        telemetry.addData("angle", Math.toDegrees(myPose_Heading+angleOfset));
        telemetry.update();
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

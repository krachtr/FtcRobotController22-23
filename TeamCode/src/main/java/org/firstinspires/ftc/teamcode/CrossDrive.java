package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class CrossDrive extends LinearOpMode {
    public static DcMotor left;
    public static DcMotor right;
    public static DcMotor front;
    public static DcMotor back;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        front = hardwareMap.get(DcMotor.class, "front");
        back = hardwareMap.get(DcMotor.class, "back");

        // Put initialization blocks here.
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        front.setDirection(DcMotorSimple.Direction.FORWARD);
        back.setDirection(DcMotorSimple.Direction.FORWARD);

        PositionTracker positionTracker = new PositionTracker();

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                drive(gamepad1.left_stick_x, gamepad1.left_stick_y, (gamepad1.right_trigger - gamepad1.left_trigger), 0.3);
                if (gamepad1.a) {
                    telemetry.addData("X", positionTracker.getFieldX());
                    telemetry.addData("Y", positionTracker.getFieldY());
                    telemetry.addData("A", positionTracker.getFieldA());
                    telemetry.addData("Front", positionTracker.getfrontChangeInMM());
                    telemetry.addData("Right", positionTracker.getRightChangeInMM());
                    telemetry.addData("Back", positionTracker.getBackChangeInMM());
                    telemetry.addData("Left", positionTracker.getLeftChangeInMM());
                    telemetry.addData("FrontTicks0", positionTracker.getFrontTicks0());
                    telemetry.addData("FrontInMM", positionTracker.getFrontInMM());
                    telemetry.addData("ChangeInA", positionTracker.getChangeInA());


                }
                // Put loop blocks here.
                telemetry.update();
            }
        }
    }
    public void drive(double x, double y, double a, double speed) {
        left.setPower((-y - a) * speed);
        right.setPower((-y + a) * speed);
        front.setPower((x + a) * speed);
        back.setPower((x - a) * speed);
    }

}

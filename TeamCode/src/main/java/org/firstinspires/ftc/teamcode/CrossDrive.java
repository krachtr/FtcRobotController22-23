package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class CrossDrive extends LinearOpMode {
    private DcMotor left;
    private DcMotor right;
    private DcMotor front;
    private DcMotor back;

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
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
        front.setDirection(DcMotorSimple.Direction.REVERSE);
        back.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                drive(gamepad1.left_stick_x, gamepad1.left_stick_y, (gamepad1.right_trigger - gamepad1.left_trigger), 0.3);
                // Put loop blocks here.
                telemetry.update();
            }
        }
    }
    public void drive(double x, double y, double a, double speed) {
        left.setPower((y + a) * speed);
        right.setPower((y - a) * speed);
        front.setPower((-x - a) * speed);
        back.setPower((-x + a) * speed);
    }


}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class IncoderCheck extends LinearOpMode {
    public static DcMotor left;
    public static DcMotor right;
    public static DcMotor front;
    public static DcMotor back;

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "right");
        right = hardwareMap.get(DcMotor.class, "left");
        front = hardwareMap.get(DcMotor.class, "front");
        back = hardwareMap.get(DcMotor.class, "back");


        // Put initialization blocks here.
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        front.setDirection(DcMotorSimple.Direction.FORWARD);
        back.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                telemetry.addData("left", left.getCurrentPosition());
                telemetry.addData("right", right.getCurrentPosition());
                telemetry.addData("front", front.getCurrentPosition());
                telemetry.addData("back", back.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="moveUpManualy", group = "b")

public class moveUpManualy extends LinearOpMode {
    public static DcMotor upMotor;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        upMotor = hardwareMap.get(DcMotor.class, "up");

        upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        upMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        upMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                upMotor.setPower(-gamepad1.right_stick_y*.2);
                telemetry.addData("grab pos", upMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}

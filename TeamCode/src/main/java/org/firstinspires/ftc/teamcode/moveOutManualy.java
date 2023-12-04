package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class moveOutManualy extends LinearOpMode {
    public static DcMotor outModer;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        outModer = hardwareMap.get(DcMotor.class, "out");

        outModer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        outModer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outModer.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                outModer.setPower(-gamepad1.right_stick_y);
                telemetry.addData("grab pos", outModer.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}

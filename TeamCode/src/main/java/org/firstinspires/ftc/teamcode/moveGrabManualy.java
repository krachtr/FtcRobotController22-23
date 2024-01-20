package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="moveGrabManualy", group = "b")

public class moveGrabManualy extends LinearOpMode {
    public static DcMotor grabModer;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        grabModer = hardwareMap.get(DcMotor.class, "grab");

        grabModer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        grabModer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabModer.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                grabModer.setPower(-gamepad1.right_stick_y);
                telemetry.addData("grab pos", grabModer.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}

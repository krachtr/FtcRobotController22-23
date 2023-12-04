package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class grabTest extends LinearOpMode {
    public static DcMotor grabModer;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int grabUpPos = 26594;
        int grabHangPos = 15661;
        int grabCamPos = 3395;
        int grabDownPos = 0;

        grabModer = hardwareMap.get(DcMotor.class, "grab");

        grabModer.setTargetPosition(grabDownPos);
        grabModer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        grabModer.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            grabModer.setPower(1);
            while (opModeIsActive()) {
                if (gamepad1.dpad_up){
                    grabModer.setTargetPosition(grabUpPos);
                } else if (gamepad1.dpad_down) {
                    grabModer.setTargetPosition(grabHangPos);
                } else if (gamepad1.dpad_left) {
                    grabModer.setTargetPosition(grabDownPos);
                } else if (gamepad1.dpad_right) {
                    grabModer.setTargetPosition(grabCamPos);
                }

                telemetry.addData("grab pos", grabModer.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}

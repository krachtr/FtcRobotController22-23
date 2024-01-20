
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "dataWrighter", group = "test")
@Disabled
public class dataWrighter extends LinearOpMode {
  int data1 = 0;

  @Override
  public void runOpMode() {
    waitForStart();
    while (opModeIsActive()) {
      if(gamepad1.a){
        data1 = 1;
      }
      if(gamepad1.b){
        data1 = 2;
      }
      telemetry.addData("data", data1);
      telemetry.update();
    }
    PoseStorage.data = data1;
  }
}

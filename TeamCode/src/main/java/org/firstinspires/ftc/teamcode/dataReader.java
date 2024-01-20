
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "dataReader", group = "test")
@Disabled
public class dataReader extends LinearOpMode {
  int data1 = 0;

  @Override
  public void runOpMode() {
    waitForStart();
    while (opModeIsActive()) {
      telemetry.addData("data", PoseStorage.data);
      telemetry.update();
    }
  }
}

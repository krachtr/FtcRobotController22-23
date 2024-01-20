/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="check", group="c")
public class checkProgram extends LinearOpMode {
    public static DcMotor left;
    public static DcMotor right;
    public static DcMotor front;
    public static DcMotor back;
    public static DcMotor inModer;
    public static DcMotor outModer;
    public static DcMotor grabModer;
    public static DcMotorEx upModer;

    public Servo shooter;

    private DistanceSensor sensorDistanceL;
    private DistanceSensor sensorDistanceR;
    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "right");
        right = hardwareMap.get(DcMotor.class, "left");
        front = hardwareMap.get(DcMotor.class, "front");
        back = hardwareMap.get(DcMotor.class, "back");

        inModer = hardwareMap.get(DcMotor.class, "in");
        outModer = hardwareMap.get(DcMotor.class, "out");
        grabModer = hardwareMap.get(DcMotor.class, "grab");
        upModer = (DcMotorEx)hardwareMap.get(DcMotor.class, "up");

        shooter = hardwareMap.get(Servo.class, "shooter");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        front.setDirection(DcMotorSimple.Direction.FORWARD);
        back.setDirection(DcMotorSimple.Direction.REVERSE);
        inModer.setDirection(DcMotorSimple.Direction.FORWARD);
        outModer.setDirection(DcMotorSimple.Direction.FORWARD);
        grabModer.setDirection(DcMotorSimple.Direction.FORWARD);
        upModer.setDirection(DcMotorSimple.Direction.FORWARD);

        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "leftdis");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "rightdist");

        Rev2mDistanceSensor sensorTimeOfFlightL = (Rev2mDistanceSensor) sensorDistanceL;
        Rev2mDistanceSensor sensorTimeOfFlightR = (Rev2mDistanceSensor) sensorDistanceR;
        waitForStart();
        left.setPower(.5);
        right.setPower(.5);
        front.setPower(.5);
        back.setPower(.5);
        upModer.setPower(.5);
        inModer.setPower(.5);
        outModer.setPower(.5);
        grabModer.setPower(.5);
        while (opModeIsActive()) {
            left.setPower(0);
            right.setPower(0);
            front.setPower(0);
            back.setPower(0);
            upModer.setPower(0);
            inModer.setPower(0);
            outModer.setPower(0);
            grabModer.setPower(0);
            telemetry.addData("Left Drive",left.getCurrentPosition());
            telemetry.addData("Right Drive",right.getCurrentPosition());
            telemetry.addData("Front Drive",front.getCurrentPosition());
            telemetry.addData("Back Drive",back.getCurrentPosition());
            telemetry.addLine("");
            telemetry.addData("In Motor",inModer.getCurrentPosition());
            telemetry.addData("Out Motor",outModer.getCurrentPosition());
            telemetry.addData("Up Motor",upModer.getCurrentPosition());
            telemetry.addData("Grab Motor",grabModer.getCurrentPosition());
            telemetry.addLine("");
            telemetry.addData("Left Distens Sensor", sensorDistanceL.getDistance(DistanceUnit.INCH)+"\"");
            telemetry.addData("Right Distens Sensor", sensorDistanceR.getDistance(DistanceUnit.INCH)+"\"");
            telemetry.update();
            if(left.getCurrentPosition()==0 || right.getCurrentPosition()==0 ||
                    front.getCurrentPosition()==0 || back.getCurrentPosition()==0 ||
                    upModer.getCurrentPosition()==0 || inModer.getCurrentPosition()==0 ||
                    outModer.getCurrentPosition()==0 || grabModer.getCurrentPosition()==0 ||
                    sensorDistanceL.getDistance(DistanceUnit.INCH)==0 || sensorDistanceR.getDistance(DistanceUnit.INCH)==0){
                telemetry.addLine("Some Thing Is Not Pluged in");
            }
        }
    }
}

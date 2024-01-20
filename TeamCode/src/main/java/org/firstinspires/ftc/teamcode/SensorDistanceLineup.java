/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates how to use the REV Robotics 2M Distance Sensor.
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.revrobotics.com/rev-31-1505/
 */
@TeleOp(name = "SensorDistanceLineUp", group = "test")
@Disabled
public class SensorDistanceLineup extends LinearOpMode {

    public static DcMotor left;
    public static DcMotor right;
    public static DcMotor front;
    public static DcMotor back;

    private DistanceSensor sensorDistanceL;
    private DistanceSensor sensorDistanceR;

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "right");
        right = hardwareMap.get(DcMotor.class, "left");
        front = hardwareMap.get(DcMotor.class, "front");
        back = hardwareMap.get(DcMotor.class, "back");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        front.setDirection(DcMotorSimple.Direction.FORWARD);
        back.setDirection(DcMotorSimple.Direction.REVERSE);

        // you can use this as a regular DistanceSensor.
        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "leftdis");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "rightdis");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlightL = (Rev2mDistanceSensor) sensorDistanceL;
        Rev2mDistanceSensor sensorTimeOfFlightR = (Rev2mDistanceSensor) sensorDistanceR;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();
        double ldis = 0;
        double rdis = 0;

        double leftPow = 0;
        double rightPow = 0;

        double oldLDis = 0;
        double oldRDis = 0;

        double target = 2.5;

        double PMultiplyer = .025;

        double IDisierd = .2;

        double maxAccsell = .1;

        waitForStart();
        while(opModeIsActive()) {
            ldis = sensorDistanceL.getDistance(DistanceUnit.INCH);
            rdis = sensorDistanceR.getDistance(DistanceUnit.INCH);
            if(gamepad1.a &&
                    ldis < 75 &&
                    Math.abs(ldis - rdis) < 5)
                        {

                    leftPow = (ldis - target) * PMultiplyer;
                    rightPow = (rdis - target) * PMultiplyer;

                    leftPow -=  (oldLDis - ldis) - IDisierd;
                    rightPow -=  (oldRDis - rdis) - IDisierd;

                    oldLDis = ldis;
                    oldRDis = rdis;

                    //    ->   v  <-  that + is not a negitive becus the power is negitive
                    if(leftPow + left.getPower() > maxAccsell){
                        leftPow = -left.getPower() + maxAccsell;
                    }

                    if(rightPow + right.getPower() > maxAccsell){
                        rightPow = -right.getPower() + maxAccsell;
                    }

                    move2(-gamepad1.left_stick_x, -leftPow, -rightPow,.3);
            } else {
                move(-gamepad1.left_stick_x, -gamepad1.left_stick_y, (gamepad1.right_trigger - gamepad1.left_trigger), 1.5);
            }
            // generic DistanceSensor methods.
            telemetry.addData("range", String.format("%.01f in", ldis));
            telemetry.addData("range", String.format("%.01f in", rdis));;

            telemetry.update();
        }
    }
    public void move(double x, double y, double a, double speed) {
        double rx=x;
        double ry=y;
        //rx = -(-(x*Math.cos(myPose.getHeading()+angleOfset)) + (y*Math.sin(myPose.getHeading()+angleOfset)));
        //ry = (y*Math.cos(myPose.getHeading()+angleOfset)) + (x*Math.sin(myPose.getHeading()+angleOfset));
        left.setPower((-ry - a) * speed);
        right.setPower((-ry + a) * speed);
        front.setPower((rx + a) * speed);
        back.setPower((rx - a) * speed);
    }
    public void move2(double x, double y, double y2, double speed) {
        double rx=x;
        double ry=y;
        //rx = -(-(x*Math.cos(myPose.getHeading()+angleOfset)) + (y*Math.sin(myPose.getHeading()+angleOfset)));
        //ry = (y*Math.cos(myPose.getHeading()+angleOfset)) + (x*Math.sin(myPose.getHeading()+angleOfset));
        left.setPower((y));
        right.setPower((y2));
        front.setPower((rx) * speed);
        back.setPower((rx) * speed);
    }
}

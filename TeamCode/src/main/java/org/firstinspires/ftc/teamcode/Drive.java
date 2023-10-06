package org.firstinspires.ftc.teamcode;

import java.util.Timer;

public class Drive extends Thread{
    double fieldX = 0;
    double fieldY = 0;
    double fieldA = 0;

    double robotXErr;
    double robotYErr;
    double AErr;

    double robotXErr1 = 0;
    double robotYErr1 = 0;
    double AErr1 = 0;

    double robotXErr0;
    double robotYErr0;
    double AErr0;

    double Vx;
    double Vy;
    double Va;

    double directToXAngle;

    double fieldXErr;
    double fieldYErr;

    Drive(){

        this.setDaemon(true);
        this.start();
    }
    @Override
    public void run() {

        //step 1
        //resolution which is measured in ticks/revolutions
        final int res = 28 * 20;
        //previous tick count
        double frontTicks0;
        double rightTicks0;
        double backTicks0;
        double leftTicks0;
        //current tick count
        double frontTicks1 = CrossDrive.front.getCurrentPosition();
        double rightTicks1 = CrossDrive.right.getCurrentPosition();
        double backTicks1 = CrossDrive.back.getCurrentPosition();
        double leftTicks1 = CrossDrive.left.getCurrentPosition();
        //change in ticks
        double frontChangeInTicks;
        double rightChangeInTicks;
        double backChangeInTicks;
        double leftChangeInTicks;
        //change in mm

        //circumference which is measured in mm/revolutions
        final double circumference = Math.PI * 90 * 1.04166666667;

        final double mmPerTick = circumference/res;

        double changeInRobotX;
        double changeInRobotY;

        double changeInFieldX;
        double changeInFieldY;

        double changeInA;

        //distens ditwean to wheel / 2 and convrted to mm
        final double distanceFromCenterToWheel = 13.5 / 2 * 25.4 * 0.95821133883;

        Timer timer = new Timer();
        while (true){
            //step 2
            frontTicks0 = frontTicks1;
            rightTicks0 = rightTicks1;
            backTicks0 = backTicks1;
            leftTicks0 = leftTicks1;

            frontTicks1 = CrossDrive.front.getCurrentPosition();
            rightTicks1 = CrossDrive.right.getCurrentPosition();
            backTicks1 = CrossDrive.back.getCurrentPosition();
            leftTicks1 = CrossDrive.left.getCurrentPosition();

            frontChangeInTicks = frontTicks1 - frontTicks0;
            rightChangeInTicks = rightTicks1 - rightTicks0;
            backChangeInTicks = backTicks1 - backTicks0;
            leftChangeInTicks = leftTicks1 - leftTicks0;

            double frontChangeInMM = frontChangeInTicks * mmPerTick;
            double rightChangeInMM = rightChangeInTicks * mmPerTick;
            double backChangeInMM = backChangeInTicks * mmPerTick;
            double leftChangeInMM = leftChangeInTicks * mmPerTick;

            changeInRobotX = (frontChangeInMM + backChangeInMM) / 2;
            changeInRobotY = (rightChangeInMM + leftChangeInMM) / 2;
            changeInA = (- frontChangeInMM - rightChangeInMM + backChangeInMM + leftChangeInMM) / 4 / distanceFromCenterToWheel;

            changeInFieldX = (changeInRobotX * Math.cos(fieldA + changeInA / 2)) + (-changeInRobotY * Math.sin(fieldA + changeInA / 2));
            changeInFieldY = (changeInRobotY * Math.cos(fieldA + changeInA / 2)) + (changeInRobotX * Math.sin(fieldA + changeInA / 2));

            fieldX += changeInFieldX;
            fieldY += changeInFieldY;
            fieldA += changeInA;
            if (fieldA >= 360){
                fieldA -= 360;
            }
            calcErr(0,0,0);
        }
    }
    public void calcErr(double X, double Y, double A){
        fieldXErr = X - fieldX;
        fieldYErr = Y - fieldY;
        AErr = A - fieldA;

        double directErr = Math.sqrt(Math.pow(fieldXErr, 2) + Math.pow(fieldYErr, 2));

        directToXAngle = Math.atan(fieldYErr/fieldXErr);
        if (fieldXErr < 0 && fieldYErr < 0){
            directToXAngle -= Math.toRadians(180);
        } else if (fieldXErr < 0 && fieldYErr >= 0) {
            directToXAngle += Math.toRadians(180);
        }

        robotXErr = Math.cos(directToXAngle - fieldA) * directErr;
        robotYErr = Math.sin(directToXAngle - fieldA) * directErr;
    }

    public void celcXYAWithPID(double X, double Y, double A){
        final double KPx = .1;
        final double KPy = .1;
        final double KPa = .1;
        final double KIx = 0;
        final double KIy = 0;
        final double KIa = 0;
        final double KDx = 0;
        final double KDy = 0;
        final double KDa = 0;

        if (robotXErr1 != 0 && robotYErr1 != 0 && AErr1 != 0) {
            robotXErr0 = robotXErr1;
            robotYErr0 = robotYErr1;
            AErr0 = AErr1;
        } else {
            robotXErr0 = robotXErr;
            robotYErr0 = robotXErr;
            AErr0 = AErr;
        }
        robotXErr1 = robotXErr;
        robotYErr1 = robotYErr;
        AErr1 = AErr;

        //System.out.println(Timer.getTime);

        Vx = KPx * (robotXErr) + KIx *  0 + KDx;
    }

    public double getFieldX() {
        return fieldX / 25.4;
    }
    public double getFieldY() {
        return fieldY / 25.4;
    }
    public double getFieldA() {
        return Math.toDegrees(fieldA);
    }
    public double getRobotXErr() {
        return robotXErr / 25.4;
    }
    public double getRobotYErr() {
        return robotYErr / 25.4;
    }
    public double getAErr() {
        return Math.toDegrees(AErr);
    }
    public double getDirectToXAngle(){
        return Math.toDegrees(directToXAngle);
    }
    public double getFieldXErr(){
        return fieldXErr / 25.4;
    }
    public double getFieldYErr(){
        return fieldYErr / 25.4;
    }

}

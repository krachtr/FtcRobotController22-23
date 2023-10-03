package org.firstinspires.ftc.teamcode;

public class PositionTracker extends Thread{
    double fieldX = 0;
    double fieldY = 0;
    double fieldA = 0;

    double frontChangeInMM;
    double rightChangeInMM;
    double backChangeInMM;
    double leftChangeInMM;

    double frontTicks0;

    double frontInMM = 0;

    double changeInA;
    PositionTracker(){

        this.setDaemon(true);
        this.start();
    }
    @Override
    public void run() {

        //step 1
        //resolution which is measured in ticks/revolutions
        final int res = 28 * 20;
        //previous tick count
        //double frontTicks0;
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
        final double circumference = Math.PI * 90;

        final double mmPerTick = circumference/res;

        double changeInRobotX;
        double changeInRobotY;

        double changeInFieldX;
        double changeInFieldY;

        //distens ditwean to wheel / 2 and convrted to mm
        final double distanceFromCenterToWheel = 13.5 / 2 * 25.4 * 1.0861;

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

            frontChangeInMM = frontChangeInTicks * mmPerTick;
            rightChangeInMM = rightChangeInTicks * mmPerTick;
            backChangeInMM = backChangeInTicks * mmPerTick;
            leftChangeInMM = leftChangeInTicks * mmPerTick;

            frontInMM += frontChangeInMM;

            changeInRobotX = (frontChangeInMM + backChangeInMM) / 2;
            changeInRobotY = (rightChangeInMM + leftChangeInMM) / 2;
            changeInA = (- frontChangeInMM - rightChangeInMM + backChangeInMM + leftChangeInMM) / 4 / distanceFromCenterToWheel;

            changeInFieldX = (changeInRobotX * Math.cos(fieldA + changeInA / 2)) + (-changeInRobotY * Math.sin(fieldA + changeInA / 2));
            changeInFieldY = (changeInRobotY * Math.cos(fieldA + changeInA / 2)) + (changeInRobotX * Math.sin(fieldA + changeInA / 2));

            fieldX += changeInFieldX;
            fieldY += changeInFieldY;
            fieldA += changeInA;
        }
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

    public double getfrontChangeInMM() {
        return frontChangeInMM;
    }

    public double getRightChangeInMM() {
        return rightChangeInMM;
    }

    public double getBackChangeInMM() {
        return backChangeInMM;
    }

    public double getLeftChangeInMM() {
        return leftChangeInMM;
    }

    public double getFrontTicks0() {
        return frontTicks0;
    }

    public double getFrontInMM() {
        return frontInMM;
    }

    public double getChangeInA() {
        return changeInA;
    }
}

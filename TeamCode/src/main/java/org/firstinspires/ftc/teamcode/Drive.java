package org.firstinspires.ftc.teamcode;

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

    double robotXErr0 = 0;
    double robotYErr0 = 0;
    double AErr0 = 0;

    double Vx;
    double Vy;
    double Va;

    double directToXAngle;

    double fieldXErr;
    double fieldYErr;

    double time0;
    double time1 = 0;

    double iXer = 0;
    double iYer = 0;
    double iAer = 0;

    double dXer;
    double dYer;

    double dAer;

    double VA0 = 0;
    double VX0 = 0;
    double VY0 = 0;


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
        final double distanceFromCenterToWheel = 13.5 / 2 * 25.4 / 0.88062622;

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
            if(fieldA > Math.toRadians(180)){
                fieldA -= Math.toRadians(360);
            } else if (fieldA < -Math.toRadians(180)) {
                fieldA += Math.toRadians(360);
            }
            calcErr(0,0,0);
            calcXYAWithPID();
        }
    }
    public void calcErr(double X, double Y, double A){
        fieldXErr = X - fieldX;
        fieldYErr = Y - fieldY;
        AErr = A - fieldA;

        double directErr = Math.sqrt(Math.pow(fieldXErr, 2) + Math.pow(fieldYErr, 2));

        if (fieldXErr != 0) {
            directToXAngle = Math.atan(fieldYErr / fieldXErr);
        } else if (fieldYErr >= 0){
            directToXAngle = Math.toRadians(90);
        } else {
            directToXAngle = Math.toRadians(-90);
        }
        if (fieldXErr < 0 && fieldYErr < 0){
            directToXAngle -= Math.toRadians(180);
        } else if (fieldXErr < 0 && fieldYErr >= 0) {
            directToXAngle += Math.toRadians(180);
        }

        robotXErr = Math.cos(directToXAngle - fieldA) * directErr;
        robotYErr = Math.sin(directToXAngle - fieldA) * directErr;

    }

    public void calcXYAWithPID(){
        final double KPx = .004;
        final double KPy = .004;
        final double KPa = 1;
        final double KIx = 0;
        final double KIy = 0;
        final double KIa = 0;
        final double KDx = 0;
        final double KDy = 0;
        final double KDa = 0;

        if (time1 != 0) {
            time0 = time1;
        } else {
            time0 = System.currentTimeMillis();
        }
        time1 = System.currentTimeMillis();

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


        iXer += (robotXErr1 + robotXErr0) / 2 * (time1 - time0);
        dXer = (robotXErr1 - robotXErr0) / (time1 - time0);
        Vx = KPx * robotXErr + KIx * iXer + KDx * dXer;


        iYer += (robotYErr1 + robotYErr0) / 2 * (time1 - time0);
        dYer = (robotYErr1 - robotYErr0) / (time1 - time0);
        Vy = KPy * robotYErr + KIy * iYer + KDy * dYer;


        iAer += (AErr1 + AErr0) / 2 * (time1 - time0);
        dAer = (AErr1 - AErr0) / (time1 - time0);
        Va = KPa * AErr + KIa * iAer + KDa * dAer;

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
    public double getVx(){
        return Vx;
    }
    public double getVy(){
        return Vy;
    }
    public double getVa(){
        return -Va;
    }
    public double getIXer(){
        return iXer;
    }
    public double getDXer(){
        return dXer;
    }

}

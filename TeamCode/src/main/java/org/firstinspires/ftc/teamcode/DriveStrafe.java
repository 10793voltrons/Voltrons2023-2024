package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
public class DriveStrafe {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    //DcMotor arm;
    Imu imu;

    //public Drive(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, DcMotor arm, BNO055IMU imu) {
    public DriveStrafe(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, Imu imu) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        //this.arm = arm;
        this.imu = imu;
    }

    public void idle() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        //arm.setPower(0);
    }

   /* public void armHold() {
        arm.setPower(1);
    }*/

    public void frontTimed(double power, int ms, double angle, double kp) {

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.milliseconds() < ms) {

            double error = Imu.getError(imu.getAngleNormalized(), angle);
            double correction = error * kp;


            leftFront.setPower(power + correction);
            leftBack.setPower(power + correction);
            rightFront.setPower(power - correction);
            rightBack.setPower(power - correction);
        }
        idle();
    }


    public void backTimed(double power, int ms, double angle, double kp) {

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.milliseconds() < ms) {

            double error = Imu.getError(imu.getAngleNormalized(), angle);
            double correction = error * kp;

            leftFront.setPower(-power + correction);
            leftBack.setPower(-power + correction);
            rightFront.setPower(-power - correction);
            rightBack.setPower(-power - correction);
        }
        idle();
    }

    public void leftTimed(double power, int ms, double angle, double kp) {

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.milliseconds() < ms) {

            double error = Imu.getError(imu.getAngleNormalized(), angle);
            double correction = error * kp;

            leftFront.setPower(-power + correction);
            leftBack.setPower(power + correction);
            rightFront.setPower(power - correction);
            rightBack.setPower(-power - correction);
        }
        idle();
    }

    public void left(double power) {

        //double error = Imu.getError(imu.getAngle(),angle);
        //double correction = error * kp;

        leftFront.setPower(-power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(-power);
        idle();
    }

    public void rightTimed(double power, int ms, double angle, double kp) {

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.milliseconds() < ms) {

            double error = Imu.getError(imu.getAngleNormalized(), angle);
            double correction = error * kp;

            leftFront.setPower(power + correction);
            leftBack.setPower(-power + correction);
            rightFront.setPower(-power - correction);
            rightBack.setPower(power - correction);
        }
        idle();
    }


    public void armUpTimed(double power, int ms) {  //Sube el slide

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.milliseconds() < ms) {

            //double error = Imu.getError(imu.getAngleNormalized(), angle);
            //double correction = error * kp;

           // arm.setPower(power);
        }
        //armHold();
    }


    public void setOrientation(double power, double angle, double timeLimit) {

        ElapsedTime time = new ElapsedTime();

        double multiplier = 1;
        time.reset();

        while (!Thread.currentThread().isInterrupted()) {

            if (time.milliseconds() > timeLimit) {
                break;
            }

            double error = Imu.getError(imu.getAngleNormalized(), angle);

            if (Math.abs(error) < 5) {
                // Romper cuando llegue a un angulo
                break;
            }

            if (Math.abs(error) < 15) {
                multiplier = 0.3;
            }

            leftBack.setPower(power * multiplier);
            leftFront.setPower(power * multiplier);
            rightFront.setPower(-power * multiplier);
            rightBack.setPower(-power * multiplier);

        }

        idle();
    }
}



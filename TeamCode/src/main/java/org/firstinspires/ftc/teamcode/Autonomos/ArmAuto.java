package org.firstinspires.ftc.teamcode.Autonomos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ArmAuto")

public class ArmAuto extends LinearOpMode {
    private DcMotorEx arm;
    private Servo graber;
    private Servo wrist;

    public static int target = 0;
    private final double ticksEnGrado = 700/180.0;
    private PIDController controller;

    public static double graberClosed = 1;
    public static double graberOpen = 0.85;
    public static double wristGrab = 1.0;
    public static double wristDrop = 0.70;

    public static double p=0, i=0, d=0;
    public static double f=0;
    public static int regreso = 0;
    public static int suelta = 0;
    public static int fin = 0;


    ElapsedTime tiempo = new ElapsedTime();
    ElapsedTime espera = new ElapsedTime();


    @Override
    public void runOpMode(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorEx.Direction.REVERSE);

        graber = hardwareMap.servo.get("graber");
        wrist = hardwareMap.servo.get("wrist");
        graber.setPosition(graberClosed);
        sleep(300);
        wrist.setPosition(wristGrab);

        target = 0;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addData("target = %i", target);
                telemetry.addData("regreso = %i", regreso);
                telemetry.addData("Posición = %f", arm.getCurrentPosition());
                telemetry.update();

                controller.setPID(p, i, d);
                int armPos = arm.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticksEnGrado)) * f;
                double power = pid + ff;
                //graber.setPosition(graberClosed);
                arm.setPower(power);

               /* if (target >= 600 && regreso == 0 && espera.milliseconds() > 5000){
                    regreso = 1;
                }
                else if (target <= 0 && regreso == 1)
                {
                    //sleep(2000);
                    regreso = 0;
                }*/

                while (target < 700 && tiempo.milliseconds() > 50 && regreso == 0) {
                    wrist.setPosition(wristDrop);
                    p=0.005;
                    i=0.005;
                    d=1E-10;
                    f=0.005;
                    target += 10;
                    if (target<700)
                        tiempo.reset();
                    else{
                        espera.reset();
                        while (espera.seconds() < 4){
                            graber.setPosition(graberOpen);
                            regreso=1;
                        }
                        tiempo.reset();
                    }
                }

                while (target > 0 && tiempo.milliseconds() > 50 && regreso == 1) {
                    graber.setPosition(graberOpen);
                    p=0.005;
                    i=0.005;
                    d=1E-10;
                    f=0.005;
                    target -= 20;
                    tiempo.reset();
                    if (target <= 0 && regreso == 1){
                        break;
                    }
                }
            }


        }
    }
}


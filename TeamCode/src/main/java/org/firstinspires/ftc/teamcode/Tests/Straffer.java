package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Straffer", group = "Auto")
@Disabled
public class Straffer extends LinearOpMode {

    // Primero declaramos todas las variables que vamos a usar
    // ( Motores, servos y temporizadores)

    DcMotor leftFront;
    DcMotor rightFront;

    DcMotor leftBack;
    DcMotor rightBack;
    DcMotorEx lift;

    Servo graber;

    Servo wrist;

    private PIDController controller;

    /** Arm **/
    public static double p=0, i=0, d=0;
    public static double f=0;
    public static int target = 0;

    private final double ticksEnGrado = 700/180.0;
    //private DcMotorEx arm;

    boolean open = false;

    public static double graberClosed = 1;
    public static double graberOpen = 0.85;

    public static double wristGrab = 1.0;
    public static double wristDrop = 0.70;

    ElapsedTime aButton = new ElapsedTime();
    ElapsedTime bButton = new ElapsedTime();
    ElapsedTime xButton = new ElapsedTime();
    ElapsedTime yButton = new ElapsedTime();
    ElapsedTime lBump = new ElapsedTime();
    ElapsedTime armDelay = new ElapsedTime();
    ElapsedTime DRight = new ElapsedTime();
    ElapsedTime DLeft = new ElapsedTime();
    ElapsedTime lbump1 = new ElapsedTime();
    ElapsedTime rbump1 = new ElapsedTime();
    ElapsedTime bump2 = new ElapsedTime();

    public static double ssArriba = 0.0;
    public static double ssAbajo = 0.50;


    @Override
    public void runOpMode() throws InterruptedException {

        // Luego las asignamos a su respectivo pedazo de hardware
        leftFront = hardwareMap.dcMotor.get("fl");
        rightFront = hardwareMap.dcMotor.get("fr");
        leftBack = hardwareMap.dcMotor.get("bl");
        rightBack = hardwareMap.dcMotor.get("br");

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setDirection(DcMotorEx.Direction.REVERSE);
        xButton.reset(); //reiniciar temporizador
        DRight.reset();
        DLeft.reset();
        lbump1.reset();
        rbump1.reset();
        bump2.reset();
        target = 0;


        graber = hardwareMap.servo.get("graber");

        wrist = hardwareMap.servo.get("wrist");


        // Invertimos los motores de fabrica
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Hacemos esto para que por defecto, cuando alguien deje de mover el stick de motor, se frenen todos los motores y no se quede patinando
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reiniciamos los temporizadores
        aButton.reset();
        xButton.reset();
        yButton.reset();
        bButton.reset();
        armDelay.reset();
        bump2.reset();

        double invert = 1;
        double adjust = 10;
        double k_p = 0.001;
        int veces=0;
        int gAbierto = 0;
        int wAbierto = 0;

        wrist.setPosition(wristGrab);
        //double position_goal = arm.getCurrentPosition();
        //double anklePosition = 0.39;

        //resetArm();

        waitForStart();
        while (opModeIsActive()) {

            // Invert puede tener dos valores: 1 o -1. Al mutiplicar invert por eso, invertimos el poder que se le debe de asignar al motor

            // Adjust es un numero que se dividira entre 10, para generar un numero decimal (e.g. 0.5) entonces al multiplicar todo el valor por este, se reducira a la mitad el poder de las llantas

            rightFront.setPower((-gamepad1.left_stick_y/1.45 - gamepad1.left_stick_x/1.45 - (gamepad1.right_stick_x/1.45 * -invert)) * (adjust / 10.0));
            leftFront.setPower((-gamepad1.left_stick_y/1.45 + gamepad1.left_stick_x/1.45 + (gamepad1.right_stick_x/1.45 * -invert)) * (adjust / 10.0));
            rightBack.setPower((-gamepad1.left_stick_y/1.45 + gamepad1.left_stick_x/1.45 - (gamepad1.right_stick_x/1.45 * -invert)) * (adjust / 10.0));
            leftBack.setPower((-gamepad1.left_stick_y/1.45 - gamepad1.left_stick_x/1.45 + (gamepad1.right_stick_x/1.45 * -invert)) * (adjust / 10.0));

            /*if (target > 280)
                target = 280;*/



            // Invert Mode
            if (gamepad1.y && yButton.milliseconds() > 500) {
                if (invert == 1) {
                    leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                    rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                    leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
                    rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
                    invert = -1;
                } else {
                    leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                    rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
                    leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
                    rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
                    invert = 1;
                }
                yButton.reset();
            }




            /** PLAYER 2 **/
            if (gamepad2.x && xButton.milliseconds() > 300) {
                if (gAbierto==1){
                    graber.setPosition(graberClosed);
                    xButton.reset();
                    gAbierto = 0;
                }else{
                    graber.setPosition(graberOpen);
                    xButton.reset();
                    gAbierto = 1;
                }
            }
/**
 if (gamepad2.y && yButton.milliseconds() > 300) {
 graber.setPosition(graberOpen);
 yButton.reset();
 }*/

            /*if(gamepad2.dpad_right && DRight.milliseconds()>300 )
            {
                p=0.005;
                i=0.005;
                d=1E-10;
                f=0.005;
                target+=30;
                DRight.reset();
                veces++;
            }

            if(gamepad2.dpad_left && DLeft.milliseconds()>300 )
            {
                p=0.01;
                i=0.01;
                d=1E-10;
                f=0.01;
                target-=30;
                DLeft.reset();
            }
*/
            if(gamepad2.right_trigger>0 && DRight.milliseconds()>100 )
            {
                p=0.005;
                i=0.005;
                d=1E-10;
                f=0.005;
                target+=20;
                DRight.reset();
                veces++;
            }

            if(gamepad2.left_trigger>0 && DLeft.milliseconds()>100 )
            {
                p=0.005;
                i=0.005;
                d=1E-10;
                f=0.005;
                target-=20;
                DLeft.reset();
            }

            if (gamepad2.a && bump2.milliseconds() > 300) {
                if (wAbierto==1){
                    wrist.setPosition(wristDrop);
                    bump2.reset();
                    wAbierto = 0;
                }else{
                    wrist.setPosition(wristGrab);
                    bump2.reset();
                    wAbierto = 1;
                }
            }

            telemetry.addData("Invert", invert);
            telemetry.addData("Grabber", graber.getPosition());
            telemetry.addData("target", target);
            telemetry.addData("Slow Mode", adjust == 4);
            telemetry.addData("Wrist position:", wrist.getPosition());
            telemetry.update();

        }
    }







}


package org.firstinspires.ftc.teamcode.Tests;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Centerstage_V2")
public class TeleOpCenterstageV2 extends LinearOpMode {

    // Primero declaramos todas las variables que vamos a usar
    // ( Motores, servos y temporizadores)

    DcMotor leftFront;
    DcMotor rightFront;

    DcMotor leftBack;
    DcMotor rightBack;
    DcMotorEx arm;
    DcMotor avion;

    Servo graber;
    Servo SFloor;
    Servo wrist;
    Servo lanzador;

    private PIDController controller;

    /** Arm **/
    public static double p=0, i=0, d=0;
    public static double f=0;
    public static int target = 0;

    private final double ticksEnGrado = 700/180.0;
    //private DcMotorEx arm;

    boolean open = false;

    public static double ssArriba = 0.0;
    public static double ssAbajo = 0.75;

    public static double graberClosed = 1;
    public static double graberOpen = 0.75;

    public static double wristGrab = 1.0;
    public static double wristDrop = 0.70;
    public static double linearSlidePowerMultiplier = 0.575;
    public static int estaArriba = 1;
    public static boolean colgado = FALSE;

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
    ElapsedTime dpad_up = new ElapsedTime();




    public static double lanzador1 = 0.0;
    public static double lanzador2 = 1.50;



    @Override
    public void runOpMode() throws InterruptedException {

        // Luego las asignamos a su respectivo pedazo de hardware
        leftFront = hardwareMap.dcMotor.get("fl");
        rightFront = hardwareMap.dcMotor.get("fr");
        leftBack = hardwareMap.dcMotor.get("bl");
        rightBack = hardwareMap.dcMotor.get("br");
        avion = hardwareMap.dcMotor.get("avion");


        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorEx.Direction.REVERSE);
        //arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setPower(0);
        xButton.reset(); //reiniciar temporizador
        DRight.reset();
        DLeft.reset();
        lbump1.reset();
        rbump1.reset();
        bump2.reset();
        target = 0;


        graber = hardwareMap.servo.get("graber");
        SFloor = hardwareMap.servo.get("ss");
        wrist = hardwareMap.servo.get("wrist");
        lanzador = hardwareMap.servo.get("lanzador");


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
        dpad_up.reset();

        double invert = 1;
        double adjust = 10;
        double k_p = 0.001;
        int veces=0;
        int gAbierto = 0;
        int wAbierto = 0;
        int lanzado = 0;
        int yabierto = 0;

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

            controller.setPID(p,i,d);
            int armPos = arm.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target/ticksEnGrado))*f;
            double power = pid+ff;


            arm.setPower(power);

            // Slow Mode
            if (gamepad1.a && aButton.milliseconds() > 300) {
                if (adjust == 10) {
                    adjust = 4;
                } else {
                    adjust = 10;
                }
                aButton.reset();
            }

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

 /*           if(gamepad1.left_bumper && lbump1.milliseconds() > 300)
            {
                SFloor.setPosition(ssArriba);
            }

            if (gamepad1.right_bumper && rbump1.milliseconds() > 300)
            {
                SFloor.setPosition(ssAbajo);
            }*/

            if (gamepad1.right_trigger > 0){
                avion.setPower(0.6);
            }
            else {
                avion.setPower(0);
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
            if (gamepad2.y && yButton.milliseconds() > 300) {
                if (yabierto==1){
                    graber.setPosition(graberOpen);
                    SFloor.setPosition(ssAbajo);
                    yButton.reset();
                    yabierto = 0;
                }else{
                    graber.setPosition(graberClosed);
                    SFloor.setPosition(ssArriba);
                    yButton.reset();
                    yabierto = 1;
                }
            }
            if (gamepad2.dpad_up && dpad_up.milliseconds() > 300) {
                if (lanzado == 0) {
                    lanzador.setPosition(lanzador2);
                    dpad_up.reset();
                    lanzado = 1;
                } else {
                    lanzador.setPosition(lanzador1);
                    dpad_up.reset();
                    lanzado = 0;
                }
            }

            if (gamepad2.b && bButton.milliseconds() > 300 && estaArriba == 1) {
                SFloor.setPosition(graberClosed);
                estaArriba = 0;
            }
            else if (gamepad2.b && bButton.milliseconds() > 300 && estaArriba == 0) {
                SFloor.setPosition(graberOpen);
                estaArriba = 1;
            }

            if(gamepad2.right_trigger>0 /*&& DRight.milliseconds()>100*/ )
            {
                arm.setPower(-linearSlidePowerMultiplier);
            }
            else if(gamepad2.left_trigger>0 /*&& DLeft.milliseconds()>100*/ )
            {
                arm.setPower(linearSlidePowerMultiplier);
            }
            else{
                arm.setPower(0);
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

            if (gamepad2.left_bumper && lBump.milliseconds() > 300){
                if (!colgado){
                    arm.setPower(0.6);
                    colgado = TRUE;
                } else {
                    arm.setPower(0);
                    colgado = FALSE;
                }

            }

            telemetry.addData("Invert", invert);
            telemetry.addData("Grabber", graber.getPosition());
            telemetry.addData("Arm position", arm.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.addData("Slow Mode", adjust == 4);
            telemetry.addData("Wrist position:", wrist.getPosition());
            telemetry.update();

        }
    }



    public void resetArm()
    {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWEncoders()
    {
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void armGotoPos(int pos)
    {
        runWEncoders();
        arm.setTargetPosition(pos);
        armPos();
        arm.setPower(0.3);
    }

    public void armStop()
    {
        arm.setPower(0);
    }

    public void armPos()
    {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


}


package org.firstinspires.ftc.teamcode.Autonomos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Imu;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Blue Left_V2")
public class Blue_Left_V2 extends LinearOpMode {

        private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

        // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
        // this is only used for Android Studio when using models in Assets.
        //CAMBIAMOS EL NOMBRE DEL MODELO
        private static final String TFOD_MODEL_ASSET = "BluePropV32023.tflite";
        // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
        // this is used when uploading models directly to the RC using the model upload interface.
        //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
        // Define the labels recognized in the model for TFOD (must be in training order!)
        private static final String[] LABELS = {
                "BlueProp",
        };

        /**
         * The variable to store our instance of the TensorFlow Object Detection processor.
         */
        private TfodProcessor tfod;

        /**
         * The variable to store our instance of the vision portal.
         */
        private VisionPortal visionPortal;


        /** declaracion de motores **/
        Imu imu;

        DcMotor leftFront;
        DcMotor leftBack;
        DcMotor rightFront;
        DcMotor rightBack;
        Servo graber;
        Servo wrist;
        DcMotorEx arm;

        public static double p=0, i=0, d=0;
        public static double f=0;
        public static int target = 0;
        public static int fin = 0;

        private final double ticksEnGrado = 700/180.0;


        Servo ss;
       // Servo graber;

        public static double graberClosed = .95;
        public static double graberOpen = 0.75;
        public static double ssArriba = 1.0;
        public static double ssAbajo = 0.75;

        public static double wristGrab = 1.0;
        public static double wristDrop = 0.70;
        public static int termina;
        public static int regreso;

        private PIDController controller;

        public static int pos;
        public static boolean mov;

        ElapsedTime tiempo = new ElapsedTime();
        ElapsedTime espera = new ElapsedTime();


        @Override
        public void runOpMode() {

            /** inicializar motores **/
            leftFront = hardwareMap.dcMotor.get("fl");
            rightFront = hardwareMap.dcMotor.get("fr");
            leftBack = hardwareMap.dcMotor.get("bl");
            rightBack = hardwareMap.dcMotor.get("br");

            controller = new PIDController(p,i,d);

            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //controller = new PIDController(p,i,d);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            arm = hardwareMap.get(DcMotorEx.class, "arm");
 //           arm.setDirection(DcMotorEx.Direction.REVERSE);
            //arm.setPower(0);
            target = 0;
            termina =0;
            regreso = 0;


            graber = hardwareMap.servo.get("graber");
            ss = hardwareMap.servo.get("ss");
            wrist = hardwareMap.servo.get("wrist");

            pos = 0;
            mov = false;
            /**POSICIONES INIOIALES SERVOS**/
            wrist.setPosition(wristGrab);
            ss.setPosition(ssAbajo);
            graber.setPosition(graberClosed);


            initTfod();

            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            waitForStart();

            if (opModeIsActive()) {
                while (opModeIsActive()) {

                    //frontPos(0.2, 400);
                    //sleep(2000);

                    telemetryTfod();

                    // Push telemetry to the Driver Station.
                    telemetry.addData("posición: %i", pos);
                    telemetry.addData("move = %b", mov);
                    telemetry.addData("target = %i", target);
                    telemetry.addData("termina = %i", termina);
                    telemetry.update();

                /*controller.setPID(p,i,d);
                int armPos = arm.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target/ticksEnGrado))*f;
                double power = pid+ff;*/

                    ss.setPosition(ssAbajo);

                    if (pos != 0)
                    {
                        if (pos == 1){ //Left marker
                            telemetry.update();
                            resetEncoders();
                            sleep(1000);
                            resetEncoders();
                            frontPos(0.4, 1250);
                            sleep(3500);
                            /** mover el servo del pixel **/
                            ss.setPosition(ssArriba);
                            sleep(1000);
                            fin = 1;
                            resetEncoders();
                            backPos(0.3, 420);
                            //le quito 200 al delay
                            sleep(1200);
                            resetEncoders();
                            turnLeftPos(0.6, 1100);
                            //le quito 300 al delay
                            sleep(1700);
                            resetEncoders();
                            //le subo 0.1 al power y le quito 1000 a los milisegundos
                            frontPos(-0.5, 1100);
                            sleep(2000);
                            wrist.setPosition(wristDrop);
                            sleep(1000);
                            resetEncoders();
                            frontPos(0.5, 235);
                            sleep(1500);
                            graber.setPosition(graberOpen);
                            sleep(500);
                            resetEncoders();
                            backPos(0.5, 200);
                            sleep(1500);
                            wrist.setPosition(wristGrab);
                            resetEncoders();
                            strLeftPos(0.6, 950);
                            sleep(1000);

                            break;
                        }
                        if (pos == 2)  //Center marker
                        {
                            telemetry.update();
                            /**Movieminto inicial del robot**/
                            resetEncoders();
                            //strLeftPos(0.3, 280);
                            //sleep(1000);
                            resetEncoders();
                            frontPos(0.4,1450);    //AQUI CAMBIO*************
                            sleep(3000);
                            /** mover el servo del pixel **/
                            //wrist.setPosition(wristDrop);
                            ss.setPosition(ssArriba);
                            graber.setPosition(graberClosed);
                            sleep(1000);
                            fin = 1;
                            resetEncoders();
                            backPos(0.3, 300);
                            sleep(1000);
                            resetEncoders();
                            turnLeftPos(0.6,1100);
                            sleep(2000);
                            resetEncoders();
                            frontPos(0.5, 1465);
                            sleep(3000);
                            wrist.setPosition(wristDrop);
                            sleep(1000);
                            resetEncoders();
                            frontPos(0.5, 245);
                            sleep(1500);
                            graber.setPosition(graberOpen);
                            sleep(500);
                            resetEncoders();
                            backPos(0.5, 200);
                            sleep(1500);
                            wrist.setPosition(wristGrab);
                            //sleep(1000);
                            resetEncoders();
                            strLeftPos(0.7, 1200);
                            sleep(1000);
                           /*while(termina != 1){
                                controller.setPID(p, i, d);
                                int armPos = arm.getCurrentPosition();
                                double pid = controller.calculate(armPos, target);
                                double ff = Math.cos(Math.toRadians(target / ticksEnGrado)) * f;
                                double power = pid + ff;
                                arm.setPower(power);
                                while (target < 700 && tiempo.milliseconds() > 50 && regreso == 0) {
                                    wrist.setPosition(wristDrop);
                                    p=0.005;
                                    i=0.005;
                                    d=1E-10;
                                    f=0.005;
                                    target += 20;
                                    if (target<700)
                                        tiempo.reset();
                                    else{
                                        espera.reset();
                                        while (espera.seconds() < 3){
                                            graber.setPosition(graberOpen);
                                            regreso = 1;
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
                                    if (target>0)
                                        tiempo.reset();
                                    else{
                                        espera.reset();
                                        while (espera.seconds() < 1.4){
                                            resetEncoders();
                                            strRightPos(0.4, 1200);
                                            sleep(1400);
                                            regreso = 1;
                                        }
                                        tiempo.reset();
                                    }
                                    wrist.setPosition(wristGrab);
                                    graber.setPosition(graberOpen);
                                    robotStop();
                                }
                            }*/
                            break;
                        }
                        if (pos == 3)  //Right Marker
                        {
                            telemetry.update();
                            resetEncoders();
                            frontPos(.5, 1250);
                            sleep(2000);
                            resetEncoders();
                            //le baje 10 al turn right
                            turnRightPos(0.4,1050);
                            sleep(2000);
                            resetEncoders();
                            frontPos(0.5,600);
                            sleep(1000);
                            resetEncoders();
                            /** mover el servo del pixel **/
                            ss.setPosition(ssArriba);
                            graber.setPosition(graberClosed);
                            //wrist.setPosition(wristDrop);
                            sleep(1000);
                            fin = 1;
                            resetEncoders();
                            backPos(-0.5, 1590);
                            sleep(3000);
                            resetEncoders();
                            turnLeftPos(0.5,2200);
                            sleep(2500);
                            resetEncoders();
                            strRightPos(0.5,200);
                            sleep(800);
                            wrist.setPosition(wristDrop);
                            sleep(1000);
                            resetEncoders();
                            frontPos(0.5, 360);
                            sleep(1500);
                            graber.setPosition(graberOpen);
                            sleep(500);
                            resetEncoders();
                            wrist.setPosition(wristGrab);
                            backPos(0.5, 200);
                            sleep(1000);
                            resetEncoders();
                            strLeftPos(0.7, 1550);
                            sleep(2000);
                            break;
                        }

                    }else{
                        telemetryTfod();
                        telemetry.update();
                        sleep(500);
                        sleep(2000);
                    }
                }
            }

            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();

        }   // end runOpMode()

        /**
         * Initialize the TensorFlow Object Detection processor.
         */
        private void initTfod() {

            // Create the TensorFlow processor by using a builder.
            tfod = new TfodProcessor.Builder()

                    // With the following lines commented out, the default TfodProcessor Builder
                    // will load the default model for the season. To define a custom model to load,
                    // choose one of the following:
                    //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                    //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                    .setModelAssetName(TFOD_MODEL_ASSET)
                    //.setModelFileName(TFOD_MODEL_FILE)

                    // The following default settings are available to un-comment and edit as needed to
                    // set parameters for custom models.
                    .setModelLabels(LABELS)
                    //.setIsModelTensorFlow2(true)
                    //.setIsModelQuantized(true)
                    //.setModelInputSize(300)
                    //.setModelAspectRatio(16.0 / 9.0)

                    .build();

            // Create the vision portal by using a builder.
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Set the camera (webcam vs. built-in RC phone camera).
            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }

            // Choose a camera resolution. Not all cameras support all resolutions.
            // builder.setCameraResolution(new Size(640, 480));

            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
            builder.enableLiveView(true);

            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

            // Choose whether or not LiveView stops if no processors are enabled.
            // If set "true", monitor shows solid orange screen if no processors enabled.
            // If set "false", monitor shows camera view without annotations.
            //builder.setAutoStopLiveView(true);

            // Set and enable the processor.
            builder.addProcessor(tfod);

            // Build the Vision Portal, using the above settings.
            visionPortal = builder.build();

            // Set confidence threshold for TFOD recognitions, at any time.
            tfod.setMinResultConfidence(0.50f);

            // Disable or re-enable the TFOD processor at any time.
            visionPortal.setProcessorEnabled(tfod, true);

        }   // end method initTfod()

        /**
         * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
         */
        private void telemetryTfod() {

            resetEncoders();
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            if (currentRecognitions.size() == 0 && mov == false) {
                strLeftPos(0.3, 540);
                sleep(1200);
                resetEncoders();
                telemetry.update();
                mov = true;

            } else {
                if (currentRecognitions.size() == 0 && mov == true)
                {
                    pos = 3;
                }
                if (currentRecognitions.size() == 1 && mov == true)
                {
                    pos = 1;
                }
                if (currentRecognitions.size() == 1 && mov == false)
                {
                    pos = 2;
                }
            }


            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                //telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());


            }   // end for() loop

        }   // end method telemetryTfod()



        /** Funciones de movimiento y encoders.
         * estas funciones permiten mover al robot por medio de envoder y el movimiento
         * simple de los motores indicando sólo el power a los motores
         * en los encoder se debe modificar la posición a la que se desea mover
         * independientemente del power a los motores**/
        public void front(double power) {
            leftFront.setPower(-power);
            leftBack.setPower(-power);
            rightFront.setPower(-power);
            rightBack.setPower(-power);
        }

        public void frontPos(double power, int posi) {
            runWEncoders();
            leftFront.setTargetPosition(-posi);
            rightFront.setTargetPosition(-posi);
            leftBack.setTargetPosition(-posi);
            rightBack.setTargetPosition(-posi);
            gotoPosition();
            leftFront.setPower(-power);
            leftBack.setPower(-power);
            rightFront.setPower(-power);
            rightBack.setPower(-power);
            //sleep(10000);
        }

        //BACK YA FUNCIONAL
        public void back(double power) {
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
        }

        public void backPos(double power, int posi) {
            runWEncoders();
            leftFront.setTargetPosition(+posi);
            rightFront.setTargetPosition(+posi);
            leftBack.setTargetPosition(+posi);
            rightBack.setTargetPosition(+posi);
            gotoPosition();
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
        }

        //RIGHT FUNCIONAL
        public void right(double power) {
            leftFront.setPower(-power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(-power);
        }

        public void leftPos(double power, int posi) {
            runWEncoders();
            leftFront.setTargetPosition(+posi);
            rightFront.setTargetPosition(-posi);
            leftBack.setTargetPosition(-posi);
            rightBack.setTargetPosition(+posi);
            gotoPosition();
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
            //sleep(10000);
        }

        //left funcional
        public void left (double power) {
            leftFront.setPower(power);
            leftBack.setPower(-power);
            rightFront.setPower(-power);
            rightBack.setPower(power);
        }

        //TurnLeft funcional
        public void turnLeft (double power) {
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(-power);
            rightBack.setPower(-power);
        }

        public void turnLeftPos (double power, int posi) {
            runWEncoders();
            leftFront.setTargetPosition(+posi);
            leftBack.setTargetPosition(+posi);
            rightFront.setTargetPosition(-posi);
            rightBack.setTargetPosition(-posi);
            gotoPosition();
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
            ///sleep(10000);
        }

        public void turnRight (double power) {
            leftFront.setPower(-power);
            leftBack.setPower(-power);
            rightFront.setPower(power);
            rightBack.setPower(power);
        }

        public void turnRightPos (double power, int posi) {
            runWEncoders();
            leftFront.setTargetPosition(-posi);
            leftBack.setTargetPosition(-posi);
            rightFront.setTargetPosition(posi);
            rightBack.setTargetPosition(posi);
            gotoPosition();
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
            ///sleep(10000);
        }

        public void strafeLeft (double power) {
            leftFront.setPower(power);
            leftBack.setPower(-power);
            rightFront.setPower(-power);
            rightBack.setPower(power);
        }

        public void strLeftPos (double power, int posi) {
            runWEncoders();
            leftFront.setTargetPosition(+posi);
            leftBack.setTargetPosition(-posi);
            rightFront.setTargetPosition(-posi);
            rightBack.setTargetPosition(+posi);
            gotoPosition();
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
        }

        public void strafeRight (double power) {
            leftFront.setPower(-power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(-power);
        }

        public void strRightPos (double power, int posi) {
            runWEncoders();
            leftFront.setTargetPosition(-posi);
            leftBack.setTargetPosition(posi);
            rightFront.setTargetPosition(posi);
            rightBack.setTargetPosition(-posi);
            gotoPosition();
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
        }



        public void robotStop() {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }

        public void resetEncoders()
        {
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void runWEncoders()
        {
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void gotoPosition()
        {
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void dejarPixel(){
            int regreso = 0;


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
            termina = 1;
        }

    }


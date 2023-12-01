package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
@Disabled
public class ArmTest extends OpMode {
    private PIDController controller;
   // private PIDController cont2;

    public static double p=0, i=0, d=0;
    public static double f=0;
    public static int target = 0;

    /** motor 2 **/
/*    public static double p2=0, i2=0, d2=0;
    public static double f2=0;
    public static int target2 = 0;*/

    private final double ticksEnGrado = 700/180.0;
    private DcMotorEx arm;
  //  private DcMotorEx lift;

    ElapsedTime xButton = new ElapsedTime();
    ElapsedTime DRight = new ElapsedTime();
    ElapsedTime DLeft = new ElapsedTime();
    ElapsedTime rBump = new ElapsedTime();
    ElapsedTime lBump = new ElapsedTime();




    @Override
    public void init()
    {
        //cont2 = new PIDController(p2,i2,d2);
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //lift = hardwareMap.get(DcMotorEx.class, "lift");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        //arm.setDirection(DcMotorSimple.Direction.REVERSE);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setDirection(DcMotorEx.Direction.REVERSE);
        xButton.reset(); //reiniciar temporizador
        DRight.reset();
        DLeft.reset();
        rBump.reset();
        lBump.reset();

    }

    @Override
    public void loop()
    {


        controller.setPID(p,i,d);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticksEnGrado))*f;
        double power = pid+ff;

        arm.setPower(power);

      /*  cont2.setPID(p2,i2,d2);
        int liftPos = lift.getCurrentPosition();
        double pid2 = cont2.calculate(liftPos, target2);
        double ff2 = Math.cos(Math.toRadians(target2/ticksEnGrado))*f2;
        double power2 = pid2+ff2;

        lift.setPower(power2);*/



        if(gamepad1.dpad_right && DRight.milliseconds()>300 )
        {
            p=0.01;
            i=0.01;
            d=1E-10;
            f=0.01;
            target+=20;
            DRight.reset();
        }

        if(gamepad1.dpad_left && DLeft.milliseconds()>300 )
        {
            p=0.01;
            i=0.01;
            d=1E-10;
            f=0.01;
            target-=20;
            DLeft.reset();
        }

  /*      if(gamepad1.left_bumper && lBump.milliseconds()>300 )
        {
            p2=0.05;
            i2=0.0;
            d2=1E-10;
            f2=0.0;
            target2+=20;
            lBump.reset();
        }

        if(gamepad1.right_bumper && rBump.milliseconds()>300 )
        {
            p2=0.05;
            i2 =0.0;
            d2=1E-10;
            f2=0.0;
            target2-=20;
            rBump.reset();
        }
*/

        telemetry.addData("pos ", armPos);
        //telemetry.addData("Pos lift", liftPos);
        telemetry.addData("target arm", target);
        telemetry.update();
    }
}

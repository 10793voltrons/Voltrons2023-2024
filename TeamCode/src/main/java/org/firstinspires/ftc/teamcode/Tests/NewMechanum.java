package org.firstinspires.ftc.teamcode.Tests;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "NewMechanum", group = "TeleOp")
@Disabled
public class NewMechanum extends LinearOpMode {
    private Motor fl, fr, bl, br;
    private MecanumDrive mDrive;
    private GamepadEx driverOp;

    ElapsedTime yButton = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        driverOp = new GamepadEx(gamepad1);
        fl = new Motor(hardwareMap, "fl" );
        fr = new Motor(hardwareMap, "fr");
        bl = new Motor(hardwareMap, "bl");
        br = new Motor(hardwareMap, "br");


        yButton.reset();
        double invert = -1;

        mDrive = new MecanumDrive(fl, fr, bl, br);
        waitForStart();
        while(opModeIsActive()){

            mDrive.driveRobotCentric(driverOp.getLeftX(), driverOp.getLeftY()*invert, driverOp.getRightX());

            if (gamepad1.y && yButton.milliseconds() > 300){
                if (invert == 1) {
                    invert = -1;
                } else {
                    invert = 1;

                }
            }

        }

    }


}

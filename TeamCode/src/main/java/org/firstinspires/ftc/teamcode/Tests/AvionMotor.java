package org.firstinspires.ftc.teamcode.Autonomos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "AvionMotor")

public class AvionMotor extends LinearOpMode {
    private DcMotor avion;

    @Override
    public void runOpMode() throws InterruptedException {
        avion = hardwareMap.dcMotor.get("avion");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0){
                avion.setPower(1);
            }
            else {
                avion.setPower(0);
            }
        }
    }
}
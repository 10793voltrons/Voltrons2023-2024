package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
public class Imu {

    private IMU imu;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    public Imu(IMU imu){
        this.imu=imu;
    }

    YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

    public double getAngle(){

        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public double getAngleNormalized() {

        double angle = orientation.getYaw(AngleUnit.DEGREES);
        if (angle <= 0) {
            return 180 + Math.abs(angle);
        } else {
            return 180 - angle;
        }
    }

    /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
     *
     * Two input parameters are required to fully specify the Orientation.
     * The first parameter specifies the direction the printed logo on the Hub is pointing.
     * The second parameter specifies the direction the USB connector on the Hub is pointing.
     * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
     */

    /* The next two lines define Hub orientation.
     * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
     *
     * To Do:  EDIT these two lines to match YOUR mounting configuration.
     */




    // Now initialize the IMU with this mounting orientation
    // Note: if you choose two conflicting directions, this initialization will cause a code exception.
    //    imu.initialize(new IMU.Parameters(orientationOnRobot));
    /* FIN DE IMU */
    //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

    /*
    private BNO055IMU imu;


    /**
     * Initializes imu class given an imu
     *
     * @param imu imu
     */
    /*
    public Imu(BNO055IMU imu) {
        this.imu = imu;
    }

    /**
     * Gets the current heading of the robot. 180 -> 0 -> -180
     *
     * @return current heading
     */

    /*
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * Gets the current heading of the robot normalized to 0 -> 360 degrees
     *
     * @return current heading normalized to 360 degrees
     */

    /*
    public double getAngleNormalized() {
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (angle <= 0) {
            return 180 + Math.abs(angle);
        } else {
            return 180 - angle;
        }
    }

    /**
     * Gets the error between two angles
     *
     * @param angle current angle
     * @param goal  goal angle
     * @return error
     */


    public static double getError(double angle, double goal) {
        double error = goal - angle;
        while (error > 180) {
            error -= 360;
        }
        while (error < -180) {
            error += 360;
        }
        return error;
    }

    /**
     * Normalizes spline angle to the form 0 -> 360
     *
     * @param angle spline angle
     * @return normalized angle
     */

    /*
    public static double normalizeSplineAngle(double angle) {
        if (angle >= 0) {
            return 180 + (90 - angle);
        } else {
            return (270 + Math.abs(angle)) % 360;
        }
    }*/

}

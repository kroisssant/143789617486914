package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
public class DriveConstants {

    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;


    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(29, 0.02, 8, 13.8);


    public static double WHEEL_RADIUS =1.9865;
    public static double GEAR_RATIO = 1;
    public static double TRACK_WIDTH = 15.74;


    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;


    public static double MAX_VEL = 40;
    public static double MAX_ACCEL = 20;
    public static double MAX_ANG_VEL = Math.toRadians(217);
    public static double MAX_ANG_ACCEL = Math.toRadians(217);


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
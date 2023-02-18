package org.firstinspires.ftc.teamcode.drive.advanced;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class meca {

    public DcMotorEx sliderLeft, sliderRight;
    public Servo claw, turn, swingLeft, swingRight;

    public int sliderNormal, sliderLow, sliderMid, sliderHigh;
    public void initMeca(HardwareMap hardwareMap) {
        sliderRight = hardwareMap.get(DcMotorEx.class, "sliderRight");
        sliderLeft = hardwareMap.get(DcMotorEx.class, "sliderLeft");

        swingLeft = hardwareMap.get(Servo.class, "swingLeft");
        swingRight = hardwareMap.get(Servo.class, "swingRight");

        claw = hardwareMap.get(Servo.class, "intake");
        turn = hardwareMap.get(Servo.class, "turn");

        sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setSwing (double target) {
        swingRight.setPosition(target);
        swingLeft.setPosition(1-target);
    }

    public void lift(int target, double velo) {
        sliderRight.setTargetPosition(target);
        sliderLeft.setTargetPosition(target);
        sliderRight.setPower(velo);
        sliderLeft.setPower(velo);
        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

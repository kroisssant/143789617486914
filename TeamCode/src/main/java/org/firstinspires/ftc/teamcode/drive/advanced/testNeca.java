package org.firstinspires.ftc.teamcode.drive.advanced;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;

@TeleOp
public class testNeca extends LinearOpMode {


    private DcMotorEx sliderRight, sliderLeft;
    private Servo butterflyRight,butterflyLeft;
    private Servo swingRight, swingLeft;
    private Servo intake, turn;

    GAMEPAD GAMEPAD1;

    @Override
    public void runOpMode() throws InterruptedException {
        sliderRight = hardwareMap.get(DcMotorEx.class, "sliderRight");
        sliderLeft = hardwareMap.get(DcMotorEx.class, "sliderLeft");

        butterflyLeft = hardwareMap.get(Servo.class, "butterflyLeft");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyRight");

        swingLeft = hardwareMap.get(Servo.class, "swingLeft");
        swingLeft.setDirection(Servo.Direction.REVERSE);
        swingRight = hardwareMap.get(Servo.class, "swingRight");

        intake = hardwareMap.get(Servo.class, "intake");
        turn = hardwareMap.get(Servo.class, "turn");

        sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        GAMEPAD1 = new GAMEPAD(this.gamepad1, telemetry);

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Slider: ", sliderLeft.getCurrentPosition());
            telemetry.addData("Swing: ", swingLeft.getPosition());
            telemetry.addData("intake: ", intake.getPosition());
            telemetry.addData("turn: ", turn.getPosition());

            setSwing(testConfig.swing);
            turn.setPosition(testConfig.turn);
            intake.setPosition(testConfig.intake);
        }
    }

    private void setSwing (double target) {
        swingRight.setPosition(target);
        swingLeft.setPosition(target);
    }

    private void lift(int target, double velo) {
        sliderRight.setTargetPosition(target);
        sliderLeft.setTargetPosition(target);
        sliderRight.setPower(velo);
        sliderLeft.setPower(velo);
        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void brat_add(double gp_y_value, double velo) {
        lift((int)(sliderLeft.getCurrentPosition()+ gp_y_value*100), velo);
    }
}

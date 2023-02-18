package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;

@TeleOp
public class teleopSimplu extends LinearOpMode {
    meca meca;
    GAMEPAD GAMEPAD1;
    GAMEPAD GAMEPAD2;
    double clawOpen = 0.1;
    double clawClosed = 0.35;
    @Override
    public void runOpMode() throws InterruptedException {
        meca = new meca();
        meca.initMeca(hardwareMap);
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        GAMEPAD1 = new GAMEPAD(this.gamepad1, telemetry);
        GAMEPAD2 = new GAMEPAD(this.gamepad2, telemetry);
        meca.setSwing(0.20);
        meca.lift(10, 0.8);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            GAMEPAD1.run();
            GAMEPAD2.run();
            if(gamepad2.a) {
                meca.setSwing(0.17);
            }
            if(gamepad2.b) {
                meca.setSwing(0.35);
            }
            if(gamepad2.x) {
                meca.setSwing(0.48);
            }
            if(gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < 0.2 && meca.sliderLeft.getCurrentPosition() < 3000 && meca.sliderLeft.getCurrentPosition() > 10) {
                meca.lift((int)(meca.sliderLeft.getCurrentPosition()- gamepad2.left_stick_y*100), 0.8);
            }
            if(GAMEPAD1.left_trigger > 0.3) {
                meca.claw.setPosition(clawClosed);
            } else if(GAMEPAD1.right_trigger > 0.3) {
                meca.claw.setPosition(clawOpen);
            }

            if (GAMEPAD1.left_bumper.value) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * 0.25,
                                -gamepad1.left_stick_x * 0.25,
                                -gamepad1.right_stick_x * 0.25
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                gamepad1.right_stick_x
                        )
                );
            }


        }
    }
}

package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class teleop extends LinearOpMode {
    mechanismFodor mechanism;
    GAMEPAD GAMEPAD1;
    GAMEPAD GAMEPAD2;
    Mode currentMode = Mode.MECANUM;
    enum Mode {
        MECANUM,
        TANK
    }


    @Override
    public void runOpMode() throws InterruptedException {
        GAMEPAD1 = new GAMEPAD(this.gamepad1, telemetry);
        GAMEPAD2 = new GAMEPAD(this.gamepad2, telemetry);
        mechanism = new mechanismFodor(hardwareMap, GAMEPAD1, GAMEPAD2, telemetry);
        // SampleTankDriveCancelable tankDrive = new SampleTankDriveCancelable(hardwareMap);
        SampleMecanumDriveCancelable mecanumDrive = new SampleMecanumDriveCancelable(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            GAMEPAD1.run();
            GAMEPAD2.run();
            mechanism.runMechanism();
            if (isStopRequested()) return;
            switch (currentMode) {
                case MECANUM:
                    // tankDrive.
                    if (GAMEPAD1.left_bumper.value) {
                        mecanumDrive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y * 0.25,
                                        -gamepad1.left_stick_x * 0.25,
                                        -gamepad1.right_stick_x * 0.25
                                )
                        );
                    } else {
                        mecanumDrive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y,
                                        -gamepad1.left_stick_x,
                                        gamepad1.right_stick_x
                                )
                        );
                    }
                    break;
//            case TANK:
//                mecanumDrive.cancelFollowing();
//                if(GAMEPAD1.left_bumper.value) {
//                    tankDrive.setWeightedDrivePower(
//                            new Pose2d(
//                                    -gamepad1.left_stick_y * 0.25,
//                                    -gamepad1.left_stick_x * 0.25,
//                                    -gamepad1.right_stick_x * 0.25
//                            )
//                    );
//                } else {
//                    tankDrive.setWeightedDrivePower(
//                            new Pose2d(
//                                    -gamepad1.left_stick_y,
//                                    -gamepad1.left_stick_x,
//                                    gamepad1.right_stick_x
//                            )
//                    );
//                }
//                break;
            }
            mecanumDrive.update();
        }
    }

}

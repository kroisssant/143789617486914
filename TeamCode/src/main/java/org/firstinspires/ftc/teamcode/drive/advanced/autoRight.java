package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "autoRight")
public class autoRight extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    double fx = 1397.213;
    double fy = 1401.501;
    double cx = 671.664;
    double cy = 350.1073;
    double tagsize = 0.23;

    int id;

    @Override
    public void runOpMode() throws InterruptedException {
        initOpenCV();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        Trajectory mers = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(30, 0, 96))
                .build();

        Trajectory p1 = drive.trajectoryBuilder(mers.end())
                .lineToLinearHeading(new Pose2d(30, 15, Math.toRadians(0)))
                .build();

        Trajectory p3 = drive.trajectoryBuilder(mers.end())
                .lineToLinearHeading(new Pose2d(30, -15, Math.toRadians(0)))

                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            if (detections != null && detections.size() != 0) {
                for (AprilTagDetection det: detections) {
                    id = det.id;
                    telemetry.addData("id", det.id);
                    telemetry.update();
                }
            }
        }
        if(isStopRequested()) {
            return;
        }
        drive.followTrajectory(mers);
        switch (id) {
            case 1:
                drive.followTrajectory(p1);
                break;
            case 2:
                break;
            case 3:
                drive.followTrajectory(p3);
                break;
        }


    }



    private void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {}
        });
    }

}
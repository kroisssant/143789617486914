//package org.firstinspires.ftc.teamcode.drive.advanced;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
//import org.firstinspires.ftc.teamcode.drive.opmode.OpenCV;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.Arrays;
//
//import static org.openftc.easyopencv.OpenCvCameraFactory.getInstance;
//
//@Autonomous
//public class AutoComunaB extends LinearOpMode {
//
//    //drivetrain
//    SampleTankDrive drive;
//
//    //servo
//
//    //motoare
//    DcMotor intake, brat;
//
//    //opencv
//    private OpenCvCamera webcam;
//    private UltimateGoalPipeline pipeline;
//
//    //opmode
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        //init
//        initHardware();
//        initOpenCv();
//        while(!opModeIsActive() && !isStopRequested()){
//            telemetry.addData("case: " , pipeline.getAnalysis());
//            telemetry.update();
//        }
//        waitForStart();
//
//        if(isStopRequested()) return;
//
//        makeCase();
////        cazulRIGHT();
////        cazulLEFT();
//    }
//
//    //alege cazul
//    private void makeCase() {
//
//        if(pipeline.getAnalysis() == UltimateGoalPipeline.UltimateGoalRings.RIGHT)
//            cazulRIGHT();
//
//        if(pipeline.getAnalysis() == UltimateGoalPipeline.UltimateGoalRings.LEFT)
//            cazulLEFT();
//
//        if(pipeline.getAnalysis() == UltimateGoalPipeline.UltimateGoalRings.MID)
//            cazulMID();
//    }
//
//
//
//    //cazuri
//
//    private void cazulRIGHT(){
//
//        Trajectory merge = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
//                .splineTo(new Vector2d(12, 5.5), Math.toRadians(15),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new TankVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .addTemporalMarker(0.01,() ->{
//                    brat_poz(270);  //2200
//                })
//                .build();
//
//
//        Trajectory back = drive.trajectoryBuilder(new Pose2d(12, 4, Math.toRadians(0)),true)
//                .splineTo(new Vector2d(0, -6), Math.toRadians(180),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new TankVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .addTemporalMarker(0.6,() ->{
//                    brat_poz(0);  //2200
//                })
//                .build();
//
//
//
//
//
//
//        sleep(2500);
//        drive.followTrajectory(merge);
//        intake.setPower(0.62);
//        sleep(500);
////        drive.turn(Math.toRadians(14));
//        intake.setPower(0);
//        drive.followTrajectory(back);
//
//
//
//
//
//    }
//
//    private void cazulMID(){
//
//        Trajectory merge = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
//                .splineTo(new Vector2d(12, 5.5), Math.toRadians(15),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new TankVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .addTemporalMarker(0.01,() ->{
//                    brat_poz(530);  //2200
//                })
//                .build();
//
//
//        Trajectory back = drive.trajectoryBuilder(new Pose2d(12, 4, Math.toRadians(0)),true)
//                .splineTo(new Vector2d(0, -6), Math.toRadians(180),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new TankVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .addTemporalMarker(0.6,() ->{
//                    brat_poz(0);  //2200
//                })
//                .build();
//
//
//
//
//
//        sleep(2500);
//        drive.followTrajectory(merge);
//        intake.setPower(0.62);
//        sleep(500);
////        drive.turn(Math.toRadians(14));
//        intake.setPower(0);
//        drive.followTrajectory(back);
//
//    }
//
//    private void cazulLEFT(){
//        Trajectory merge = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
//                .splineTo(new Vector2d(12, 5.5), Math.toRadians(15),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new TankVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .addTemporalMarker(0.01,() ->{
//                    brat_poz(810);  //2200
//                })
//                .build();
//
//
//        Trajectory back = drive.trajectoryBuilder(new Pose2d(12, 4, Math.toRadians(0)),true)
//                .splineTo(new Vector2d(0, -6), Math.toRadians(180),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new TankVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(60)
//                )
//                .addTemporalMarker(0.6,() ->{
//                    brat_poz(0);  //2200
//                })
//                .build();
//
//
//
//
//        sleep(2500);
//        drive.followTrajectory(merge);
//        intake.setPower(0.62);
//        sleep(500);
////        drive.turn(Math.toRadians(14));
//        intake.setPower(0);
//        drive.followTrajectory(back);
//
//
//    }
//
//    //inituri
//    private void initOpenCv(){
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam =  getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"),cameraMonitorViewId);
//        pipeline = new UltimateGoalPipeline(telemetry);
//        webcam.setPipeline(pipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//            }
//            public void onError(int errorCode) {
//
//            }
//        });
//    }
//    private void initHardware(){
//
//        drive = new SampleTankDrive(hardwareMap);
//
//        intake = hardwareMap.get(DcMotor.class,"intake");
//        brat = hardwareMap.get(DcMotor.class, "brat");
//
//
//
//
//
//
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//
//
//
//    }
//
//
//
//    private void brat_poz( int pozitie){
//        brat.setTargetPosition( pozitie );
//        brat.setPower(1);
//        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//
//    //    clasa openCv
//    public static class UltimateGoalPipeline extends OpenCvPipeline
//    {
//        Telemetry telemetry;
//
//        public UltimateGoalPipeline(Telemetry telemetry){
//            this.telemetry = telemetry;
//
//        }
//
//        public enum UltimateGoalRings
//        {
//            LEFT,
//            MID,
//            RIGHT
//        }
//
//
//        static final Scalar RED = new Scalar(255, 0, 0);
//        static final Scalar BLACK = new Scalar(0, 0, 0);
//        static final Scalar WHITE = new Scalar(255, 255, 255);
//
//
//        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(910,390);
//        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(680,390);
//        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(450,390);
//
//
//
//
//        static final int REGION_WIDTH = 100;
//        static final int REGION_HEIGHT = 100;
//        static final int FULL_THRESHOLD = 105; // 93    124 for ZERO
//
//
//        Point region1_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//
//
//        Point region2_pointA = new Point(
//                REGION2_TOPLEFT_ANCHOR_POINT.x,
//                REGION2_TOPLEFT_ANCHOR_POINT.y);
//        Point region2_pointB = new Point(
//                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//
//
//        Point region3_pointA = new Point(
//                REGION3_TOPLEFT_ANCHOR_POINT.x,
//                REGION3_TOPLEFT_ANCHOR_POINT.y);
//        Point region3_pointB = new Point(
//                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//
//
//
//        Mat region1_Cb,region2_Cb,region3_Cb;
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//        int avg1, avg2, avg3;
//
//        private volatile UltimateGoalRings position = UltimateGoalRings.RIGHT;
//
//        void inputToCb(Mat input)
//        {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb );
//            Core.extractChannel(YCrCb, Cb, 2);
//        }
//
//        @Override
//        public void init(Mat firstFrame)
//        {
//            inputToCb(firstFrame);
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
//            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
//
//
//        }
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            inputToCb(input);
//
//            avg1 = (int) Core.mean(region1_Cb).val[0];
//            avg2 = (int) Core.mean(region2_Cb).val[0];
//            avg3 = (int) Core.mean(region3_Cb).val[0];
////            telemetry.addData("avg: ",avg);
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    RED, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region2_pointA, // First point which defines the rectangle
//                    region2_pointB, // Second point which defines the rectangle
//                    RED, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//            /*
//             * Draw a rectangle showing sample region 3 on the screen.
//             * Simply a visual aid. Serves no functional purpose.
//             */
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region3_pointA, // First point which defines the rectangle
//                    region3_pointB, // Second point which defines the rectangle
//                    RED, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//
//
//
//            int minOneTwo = Math.min(avg1, avg2);
//            int min = Math.min(minOneTwo, avg3);
//
//
//
//            if(min == avg1) // Was it from region 1?
//            {
//                position = UltimateGoalRings.LEFT; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region1_pointA, // First point which defines the rectangle
//                        region1_pointB, // Second point which defines the rectangle
//                        BLACK, // The color the rectangle is drawn in
//                        -1); // Negative thickness means solid fill
//            }
//            else if(min == avg2) // Was it from region 2?
//            {
//                position = UltimateGoalRings.MID; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region2_pointA, // First point which defines the rectangle
//                        region2_pointB, // Second point which defines the rectangle
//                        BLACK, // The color the rectangle is drawn in
//                        -1); // Negative thickness means solid fill
//            }
//            else if(min == avg3) // Was it from region 3?
//            {
//                position = UltimateGoalRings.RIGHT; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region3_pointA, // First point which defines the rectangle
//                        region3_pointB, // Second point which defines the rectangle
//                        BLACK, // The color the rectangle is drawn in
//                        -1); // Negative thickness means solid fill
//            }
//
//
//
//
//
//
//
////            if(avg <= FOUR_RING_THRESHOLD)
////            {
////                position = UltimateGoalRings.FOUR;
////
////            }
////            else if(avg <= ONE_RING_THRESHOLD)
////            {
////                position = UltimateGoalRings.ONE;
////
////            }
////            else
////            {
////                position = UltimateGoalRings.ZERO;
////            }
//
//            return input;
//        }
//
//        public UltimateGoalRings getAnalysis()
//        {
//            return position;
//        }
//    }
//
//}
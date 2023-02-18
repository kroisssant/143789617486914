package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;


public class Mechanism {
    private DcMotor brat;
    private DcMotor intake;
    private DcMotorSimple duck;

    private Servo servo;
    private CRServo tureta,nivel,ruleta;

    private GAMEPAD gamepad1;
    private GAMEPAD gamepad2;
    private Telemetry telemetry;



    // Copy your PID Coefficients here
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.004, 0, 0.0002);

    // Copy your feedforward gains here
    public static double kV = 0.000427;
    public static double kA = 0.000366;
    public static double kStatic = 0;

    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect
    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    // Our velocity controller
    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);



    public Mechanism(HardwareMap hardwareMap, GAMEPAD gamepad1 ,GAMEPAD gamepad2, Telemetry telemetry){
        //public Mechanism(HardwareMap hardwareMap, GAMEPAD gamepad1, Telemetry telemetry){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        initMechanism(hardwareMap);
    }

    private void initMechanism(HardwareMap hardwareMap){

        brat = hardwareMap.get(DcMotorEx.class, "brat");
        intake = hardwareMap.get(DcMotor.class, "intake");
        duck = hardwareMap.get(DcMotorSimple.class, "duck");

        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.get(Servo.class , "servo");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);


        ruleta = hardwareMap.get(CRServo.class , "ruleta");
        tureta = hardwareMap.get(CRServo.class , "tureta");
        nivel = hardwareMap.get(CRServo.class , "nivel");




//        FTC-Chineese
//        intake.setDirection(DcMotorSimple.Direction.REVERSE);



        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }

    public void mechanism(){
        intake();
        bratucu();
        lasa();
        tureta();
        rata();
        telemetry.addData("brat: ",brat.getCurrentPosition() );
        telemetry.update();



//        telemetry.addData("velocity: ", );


    }
    private void rata(){
        if(gamepad2.right_bumper.value)
            duck.setPower(0.7);
        else  if(gamepad2.left_bumper.value)
            duck.setPower(-0.7);
        else duck.setPower(0);
    }
    private void lasa(){
        if(gamepad1.b.toggle || gamepad2.b.toggle)
            servo.setPosition(1);
        else
            servo.setPosition(0);
    }
    private void bratucu(){
        if    (gamepad2.a.value) {
            brat.setTargetPosition(0);
            brat.setPower(-1);
            brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(gamepad2.y.value){
            brat.setTargetPosition(270); //800 //300
            brat.setPower(1);
            brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(gamepad2.x.value){
            brat.setTargetPosition(870);
            brat.setPower(1);
            brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(gamepad2.b.value){
            brat.setTargetPosition(580);
            brat.setPower(1);
            brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(gamepad1.dpad_down.value){
            brat.setTargetPosition(brat.getCurrentPosition()-69);
            brat.setPower(1);
            brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }
        else if(gamepad1.dpad_up.value){
            brat.setTargetPosition(brat.getCurrentPosition()+69);

            brat.setPower(-1);
            brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }


    private void tureta(){
           double viteza;
        if(gamepad1.right_bumper.toggle)
            viteza=1;
        else viteza=0.2;
            if(gamepad1.a.value) {
                nivel.setPower(viteza);
            }
            else if(gamepad1.y.value){
                nivel.setPower(-viteza);
            }
            else{
                nivel.setPower(0);
            }


            if(gamepad1.x.value) {
                tureta.setPower(viteza);
            }
            else if(gamepad1.b.value){
                tureta.setPower(-viteza);
            }
            else{
                tureta.setPower(0);
            }


            if(gamepad1.right_trigger > 0.3) {
                ruleta.setPower(viteza);
            }
            else if(gamepad1.left_trigger > 0.3){
                ruleta.setPower(-viteza);
            }
            else{
                ruleta.setPower(0);
            }

        }


    private void intake(){
        if(gamepad2.right_trigger > 0.3)
            intake.setPower(1);
        else if(gamepad2.left_trigger> 0.3)
            intake.setPower(-0.53);
        else
            intake.setPower(0);
    }










}
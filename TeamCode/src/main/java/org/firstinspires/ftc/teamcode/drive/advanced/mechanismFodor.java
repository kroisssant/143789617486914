package org.firstinspires.ftc.teamcode.drive.advanced;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;

public class mechanismFodor {
        private DcMotorEx sliderRight, sliderLeft;
        private Servo swingRight, swingLeft;
        private Servo intake, turn;

        private GAMEPAD gamepad1;
        private GAMEPAD gamepad2;

        STATES mecaState = STATES.LOW;
        STATES stateReq = STATES.nNULL;

        private Telemetry telemetry;
        double swingIntake = 0.15;
        double intakeClose = 0.3, intakeOpen = 0.1;
        boolean fs = false;//bool for front and back for turn(servo) false = front true = back

        private void initMechanism(HardwareMap hardwareMap) {
                sliderRight = hardwareMap.get(DcMotorEx.class, "sliderRight");
                sliderLeft = hardwareMap.get(DcMotorEx.class, "sliderLeft");

                swingLeft = hardwareMap.get(Servo.class, "swingLeft");
                swingRight = hardwareMap.get(Servo.class, "swingRight");

                intake = hardwareMap.get(Servo.class, "intake");
                turn = hardwareMap.get(Servo.class, "turn");

                sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                sliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                swingLeft.setDirection(Servo.Direction.REVERSE);



        }
        public mechanismFodor(HardwareMap hardwareMap, GAMEPAD gamepad1, GAMEPAD gamepad2, Telemetry telemetry) {
                this.telemetry = telemetry;
                this.gamepad1 = gamepad1;
                this.gamepad2 = gamepad2;
                initMechanism(hardwareMap);
        }

        public void runMechanism() throws InterruptedException {

                intake();
                setState();
                setFataSpate();
                bratManual();

                telemetry.addData("SliderPos: ", sliderRight.getCurrentPosition());
                telemetry.addData("Swing: ", swingLeft.getPosition());
                telemetry.addData("Case: ", mecaState);
                telemetry.addData("CaseReq: ", stateReq);
                telemetry.addData("fs: ", fs);
                telemetry.update();
        }

        private void intake() throws InterruptedException {
                if(gamepad1.left_trigger > 0.3 && mecaState == STATES.DEFAULT) {
                        // this is for picking up the cone and picks up the cone
                        intake.setPosition(intakeClose);

                } else if(gamepad1.right_trigger > 0.3 && mecaState == STATES.DEFAULT) {
                        // this set everything back up
                        intake.setPosition(intakeOpen);
                } else if(gamepad1.right_trigger > 0.3 && mecaState != STATES.DEFAULT) {
                        // TODO value for this shit
                        setSwing(swingLeft.getPosition() + 0.1);
                        Thread.sleep(300);
                        intake.setPosition(intakeOpen);
                }
        }

        private void setState() throws InterruptedException {
                // this set the state to one of the three, actionted by dirver2
                if(gamepad2.a.value) {
                        stateReq = STATES.LOW;
                }
                else if(gamepad2.b.value) {
                        stateReq = STATES.MID;
                }
                else if(gamepad2.y.value) {
                        stateReq = STATES.HIGH;
                }
                // in case that driver1 set the system to go to a state this code it for it
                if(gamepad1.right_bumper.toggle) {
                        switch (stateReq) {
                                case LOW:
                                        lift(0, 0.5);
                                        mecaState = STATES.LOW;
                                        stateReq = STATES.nNULL;
                                        break;
                                case MID:
                                        lift(500, 0.5);
                                        mecaState = STATES.MID;
                                        stateReq = STATES.nNULL;
                                        break;
                                case HIGH:
                                        lift(1000, 0.5);
                                        mecaState = STATES.HIGH;
                                        stateReq = STATES.nNULL;
                                        break;
                                default:
                                        break;
                        }
                // this sets system back to default position
                } else if(!gamepad1.right_bumper.toggle && mecaState != STATES.DEFAULT) {
                        lift(0, 0);
                        stateReq = STATES.nNULL;
                        mecaState = STATES.DEFAULT;
                }

        }

        private void setFataSpate() {
                if(gamepad2.right_bumper.toggle && !(gamepad1.right_trigger > 0.3)) {
                        switch (mecaState) {
                                case LOW:
                                        setIntake(0.3, 0);
                                        break;
                                case MID:
                                        setIntake(0.2, 0);
                                        break;
                                case HIGH:
                                        setIntake(0.2, 0);
                                        break;
                                default:
                                        setIntake(0.15, 0);
                                        break;

                        }
                        fs = false;
                } else if(!gamepad2.right_bumper.toggle && !(gamepad1.right_trigger > 0.3)) {
                        switch (mecaState) {
                                case LOW:
                                        setIntake(0.7, 0.7);
                                        break;
                                case MID:
                                        setIntake(0.8, 0.7);
                                        break;
                                case HIGH:
                                        setIntake(0.8, 0.7);
                                        break;
                                default:
                                        setIntake(0.15, 0.7);
                        }
                        fs = true;
                }
        }

        private void bratManual() {
                if(gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < 0.2) {
                        brat_add(gamepad2.left_stick_y.doubleValue(), 0.8);
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

        private void setIntake(double targerSwing, double targetTurn) {
                setSwing(targerSwing);
                turn.setPosition(targetTurn);
        }

        private void brat_add(double gp_y_value, double velo) {
                lift((int)(sliderLeft.getCurrentPosition()+ gp_y_value*100), velo);
        }

        enum STATES {
                HIGH,
                MID,
                LOW,
                DEFAULT,
                nNULL
        }
}

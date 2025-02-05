package org.firstinspires.ftc.teamcode.competitional.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.configurations.RobotConfig;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

@Config
@TeleOp(name = "🐙 Kraken 🐙", group = "A. Competitional")
public class Kraken extends LinearOpMode {

    private RobotConfig robotConfig;
    private FtcDashboard dashboard;

    private boolean isAreSlidesStopped = true;
    private boolean areSlidesDown = true;
    private boolean da = false;
    private boolean pickupIntakeButtonPressed = false;
    private boolean outtakeClawClosed = true;
    private boolean outtakeClawOpened = true;
    private boolean intakeClawClosed = true;
    private boolean intakeDown = false;
    private boolean rot0 = true;

    private PIDFController pidfControllerUp;
    private PIDFController pidfControllerDown;

    public static double P = 0.0128; //was 0.0128
    public static double I = 0;
    public static double D = 0.00005; //was 0.00005
    public static double F = 0.01;    //was                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  ;      //was 0.01
    public static double K = 0;

    public static double targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;

    private enum OuttakeState {
        START,
        INIT,
        PICKUP,
        PLACE_SAMPLE,
        ROTATE_SAMPLE,
        DROP_SAMPLE,
        PICKUP_SPECIMENE,
        PLACE_SPECIMENE
    }

    private enum IntakeState {
        START,
        INIT,
        LIFT_EXTEND,
        PICKUP,
        TRANSFER,
        WAITING,
        RETRACT
    }

    private ElapsedTime TransferTimer = new ElapsedTime();
    private ElapsedTime OuttakeServoTimer = new ElapsedTime();
    private ElapsedTime outtakeClawServoTimer = new ElapsedTime();
    private ElapsedTime yButtonPressed = new ElapsedTime();
    private ElapsedTime specimenTimer = new ElapsedTime();
    private OuttakeState currentOuttakeState = OuttakeState.START;
    private IntakeState currentIntakeState = IntakeState.START;

    @Override
    public void runOpMode() {
        robotConfig = new RobotConfig(hardwareMap);

        resetServosToInit();

        MotorConfigurationType motorConfigurationTypeFrontLeftMotor = RobotConfig.frontLeftMotor.getMotorType().clone();
        motorConfigurationTypeFrontLeftMotor.setAchieveableMaxRPMFraction(1.0);
        RobotConfig.frontLeftMotor.setMotorType(motorConfigurationTypeFrontLeftMotor);

        MotorConfigurationType motorConfigurationTypeBackLeftMotor = RobotConfig.backLeftMotor.getMotorType().clone();
        motorConfigurationTypeBackLeftMotor.setAchieveableMaxRPMFraction(1.0);
        RobotConfig.backLeftMotor.setMotorType(motorConfigurationTypeBackLeftMotor);

        MotorConfigurationType motorConfigurationTypeFrontRightMotor = RobotConfig.frontRightMotor.getMotorType().clone();
        motorConfigurationTypeFrontRightMotor.setAchieveableMaxRPMFraction(1.0);
        RobotConfig.frontRightMotor.setMotorType(motorConfigurationTypeFrontRightMotor);

        MotorConfigurationType motorConfigurationTypeBackRightMotor = RobotConfig.backRightMotor.getMotorType().clone();
        motorConfigurationTypeBackRightMotor.setAchieveableMaxRPMFraction(1.0);
        RobotConfig.backRightMotor.setMotorType(motorConfigurationTypeBackRightMotor);


        MotorConfigurationType motorConfigurationTypeUpMotor = RobotConfig.upMotor.getMotorType().clone();
        motorConfigurationTypeUpMotor.setAchieveableMaxRPMFraction(1.0);
        RobotConfig.upMotor.setMotorType(motorConfigurationTypeUpMotor);

        MotorConfigurationType motorConfigurationTypeMidMotor = RobotConfig.midMotor.getMotorType().clone();
        motorConfigurationTypeMidMotor.setAchieveableMaxRPMFraction(1.0);
        RobotConfig.midMotor.setMotorType(motorConfigurationTypeMidMotor);

        MotorConfigurationType motorConfigurationTypeDownMotor = RobotConfig.downMotor.getMotorType().clone();
        motorConfigurationTypeDownMotor.setAchieveableMaxRPMFraction(1.0);
        RobotConfig.downMotor.setMotorType(motorConfigurationTypeDownMotor);



        updatePIDFController();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = dashboard.getTelemetry();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            pidfControllerUp.setTargetPosition(targetPosition);
            pidfControllerUp.updatePosition(robotConfig.upMotor.getCurrentPosition());

//            pidfControllerDown.setTargetPosition(targetPosition);
//            pidfControllerDown.updatePosition(robotConfig.downMotor.getCurrentPosition());

            double powerUp = pidfControllerUp.runPIDF() + K;
//            double powerDown = pidfControllerDown.runPIDF() + K;

            if (Math.abs(robotConfig.upMotor.getCurrentPosition() - targetPosition) <= OuttakeConstants.TOLERANCE) {
                powerUp = 0;
            }

//            if (Math.abs(robotConfig.downMotor.getCurrentPosition() - targetPosition) <= OuttakeConstants.TOLERANCE) {
//                powerDown = 0;
//            }
            if(robotConfig.upMotor.getCurrentPosition()>10)
                areSlidesDown = false;
            if(robotConfig.upMotor.getVelocity()<0.1 && robotConfig.upMotor.getCurrentPosition()<10){
                RobotConfig.upMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                RobotConfig.midMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                RobotConfig.downMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                RobotConfig.upMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                RobotConfig.midMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                RobotConfig.downMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                areSlidesDown = true;
            }
            if(targetPosition == OuttakeConstants.OUTTAKE_MIN_POSITION && areSlidesDown){
                robotConfig.upMotor.setPower(0);
                robotConfig.midMotor.setPower(0);
                robotConfig.downMotor.setPower(0);
            }
            if(targetPosition == OuttakeConstants.OUTTAKE_MIN_POSITION && robotConfig.upMotor.getCurrentPosition()>10){
                robotConfig.upMotor.setPower(-0.9);
                robotConfig.midMotor.setPower(-0.9);
                robotConfig.downMotor.setPower(-0.9);

            }
            else {
                robotConfig.upMotor.setPower(powerUp);
                robotConfig.midMotor.setPower(powerUp);
                robotConfig.downMotor.setPower(powerUp);
            }
            robotConfig.upMotor.setPower(powerUp);
            robotConfig.midMotor.setPower(powerUp);
            robotConfig.downMotor.setPower(powerUp);
            if(-gamepad1.left_stick_y > 0.1) {
                targetPosition +=10;
            }
            else if(-gamepad1.left_stick_y < -0.1) {
                targetPosition -=10;
            }

            if(gamepad2.y) {
                targetPosition = OuttakeConstants.OUTTAKE_HANG_EXTENDED_POSITION;
            }
            if(gamepad2.a) {
                targetPosition = OuttakeConstants.OUTTAKE_HANG_RETRACTED_POSITION;
            }

            double y = -gamepad2.left_stick_y;
            double x = gamepad2.left_stick_x;
            double rx = gamepad2.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robotConfig.setMotorPowers(
                    frontLeftPower,
                    backLeftPower,
                    frontRightPower,
                    backRightPower
            );

            if (gamepad1.right_stick_button) {
                setIntakeState(IntakeState.START);
            }
            if (gamepad1.left_stick_button) {
                setOuttakeState(OuttakeState.PLACE_SAMPLE);
            }

            controlOuttakeState();
            controlIntakeState();

            updatePIDFController();

            dashboard = FtcDashboard.getInstance();
            dashboard.setTelemetryTransmissionInterval(25);
            telemetry = dashboard.getTelemetry();
            /*
            telemetry.addData("Outtake Claw Position", robotConfig.outtakeClawServo.getPosition());
            telemetry.addData("Outtake Wrist Rot Position", robotConfig.outtakeWristRotServo.getPosition());
            telemetry.addData("Outtake Wrist Y Position", robotConfig.outtakeWristYServo.getPosition());
            telemetry.addData("Outtake Elbow Right Position", robotConfig.outtakeElbowRightServo.getPosition());
            telemetry.addData("Outtake Elbow Left Position", robotConfig.outtakeElbowLeftServo.getPosition());
            telemetry.addData("TranferTimer", TransferTimer.seconds());
            telemetry.addData("Intake Elbow Right Position", robotConfig.intakeElbowRightServo.getPosition());
            telemetry.addData("Intake Elbow Left Position", robotConfig.intakeElbowLeftServo.getPosition());
            telemetry.addData("Intake Wrist Position", robotConfig.intakeWristServo.getPosition());
            telemetry.addData("Intake Wrist Right Position", robotConfig.intakeWristRightServo.getPosition());
            telemetry.addData("Intake Wrist Left Position", robotConfig.intakeWristLeftServo.getPosition());
            telemetry.addData("Intake Claw Position", robotConfig.intakeClawServo.getPosition());
            telemetry.addData("Intake Wrist Rot Position", robotConfig.intakeWristRotServo.getPosition());
            telemetry.addData("Outtake State", currentOuttakeState);
            telemetry.addData("Intake State", currentIntakeState);
            telemetry.addData("Outtake Claw Closed", outtakeClawClosed);
            telemetry.addData("Intake Claw Closed", intakeClawClosed);
           */
            telemetry.addData("velocity", RobotConfig.upMotor.getVelocity());
            telemetry.addData("da", da);
            telemetry.addData("areSlidesDown", areSlidesDown);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Left Motor Position", robotConfig.upMotor.getCurrentPosition());


            telemetry.addData("Left Motor Error", robotConfig.upMotor.getCurrentPosition() - targetPosition);
            telemetry.addData("Right Motor Error", robotConfig.downMotor.getCurrentPosition() - targetPosition);

            telemetry.addData("Right Motor Power", robotConfig.downMotor.getPower());
            telemetry.addData("Left Motor Power", robotConfig.upMotor.getPower());
            telemetry.update();
        }
    }

    private void controlOuttakeState() {
        switch (currentOuttakeState) {
            case START:
                if(gamepad1.dpad_down) {
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
                    );
                }
                if(gamepad2.left_bumper){
                    setOuttakeState(OuttakeState.PICKUP_SPECIMENE);
                }
                break;
            case INIT:
                robotConfig.setOuttakeServoPositions(
                        ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                        ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                        ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
                );
                if(gamepad2.left_bumper){
                    areSlidesDown = false;
                    setOuttakeState(OuttakeState.PICKUP_SPECIMENE);
                }
                if(gamepad1.dpad_down) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                }
                else if(gamepad1.dpad_up) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_TOP_SAMPLE_BOX;
                }
                else if(gamepad1.dpad_right) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR ;
                }
                else if(gamepad1.dpad_left) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_BOTTOM_SAMPLE_BOX;
                }
                break;
            case PICKUP:
                robotConfig.setOuttakeServoPositions(
                        ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                        ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                        ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
                );
                if(TransferTimer.seconds() > 1.0) { //micsorat
                    outtakeClawServoTimer.reset();
                    if (ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION - robotConfig.intakeElbowRightServo.getPosition() <= 0) {
                        robotConfig.setOuttakeServoPositions(
                                ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                                ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                                ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
                        );
                        setIntakeState(IntakeState.START);
                        sleep(100);
                        setOuttakeState(OuttakeState.PLACE_SAMPLE);
                    }
                }
                if(gamepad2.left_bumper){
                    areSlidesDown = false;
                    setOuttakeState(OuttakeState.PICKUP_SPECIMENE);
                }
                break;
            case PLACE_SAMPLE:
                if(gamepad1.left_bumper && OuttakeServoTimer.milliseconds() > 150) {
                    OuttakeServoTimer.reset();
                    setOuttakeState(OuttakeState.ROTATE_SAMPLE);
                }
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_POSITION
                    );
                if(gamepad1.dpad_down) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                }
                else if(gamepad1.dpad_up) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_TOP_SAMPLE_BOX;
                }
                else if(gamepad1.dpad_right) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR ;
                }
                else if(gamepad1.dpad_left) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_BOTTOM_SAMPLE_BOX;
                }
                if(gamepad1.left_trigger > 0.1){
                    setOuttakeState(OuttakeState.DROP_SAMPLE);
                    outtakeClawServoTimer.reset();
                }
                if(gamepad2.left_bumper){
                    areSlidesDown = false;
                    setOuttakeState(OuttakeState.PICKUP_SPECIMENE);
                }
                break;
            case ROTATE_SAMPLE:
                if(gamepad1.left_bumper && OuttakeServoTimer.milliseconds() > 150) {
                    OuttakeServoTimer.reset();
                    setOuttakeState(OuttakeState.PLACE_SAMPLE);
                }
                robotConfig.setOuttakeServoPositions(
                        ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                        ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                        ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_POSITION
                );
                if(gamepad1.dpad_down) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                }
                else if(gamepad1.dpad_up) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
                }
                else if(gamepad1.dpad_right) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_TOP_SAMPLE_BOX;
                }
                else if(gamepad1.dpad_left) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_BOTTOM_SAMPLE_BOX;
                }
                if(gamepad1.left_trigger > 0.1){
                    setOuttakeState(OuttakeState.DROP_SAMPLE);
                    outtakeClawServoTimer.reset();
                }
                if(gamepad2.left_bumper){
                    setOuttakeState(OuttakeState.PICKUP_SPECIMENE);
                }
                break;
            case DROP_SAMPLE:
                if(gamepad1.left_trigger > 0.1){
                    robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                }
                if(gamepad1.dpad_down) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                    setOuttakeState(OuttakeState.INIT);
                }
                else if(gamepad1.dpad_up) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
                }
                else if(gamepad1.dpad_right) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_TOP_SAMPLE_BOX;
                }
                else if(gamepad1.dpad_left) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_BOTTOM_SAMPLE_BOX;
                }

                if(gamepad2.left_bumper){
                    setOuttakeState(OuttakeState.PICKUP_SPECIMENE);
                }
                break;
            case PICKUP_SPECIMENE:
                if(outtakeClawOpened) {
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_PICKUP_SPECIMEN_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_SPECIMEN_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_SPECIMEN_POSITION
                    );
                }
                if(gamepad2.right_trigger>0.1 && outtakeClawOpened){
                    outtakeClawOpened = false;
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_PICKUP_SPECIMEN_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_SPECIMEN_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_SPECIMEN_POSITION
                    );
                }
                if(gamepad2.dpad_up && !outtakeClawOpened){
                    outtakeClawOpened = true;
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_PICKUP_SPECIMEN_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_SPECIMEN_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_SPECIMEN_POSITION
                    );
                }

                if(gamepad2.right_bumper) {
                    setOuttakeState(OuttakeState.PLACE_SPECIMENE);
                }
                if(gamepad1.dpad_down) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                }
                break;
            case PLACE_SPECIMENE:
                if(!outtakeClawOpened) {
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION
                    );
                }
                if(gamepad2.left_trigger>0.1){
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR;
                }
                if(gamepad2.right_trigger>0.1){
                    outtakeClawOpened = true;
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION
                    );
                    targetPosition = 320;
                }

                if(gamepad2.right_bumper) {
                    setOuttakeState(OuttakeState.PLACE_SPECIMENE);
                }
                if(gamepad2.left_bumper && outtakeClawOpened){
                    setOuttakeState(OuttakeState.PICKUP_SPECIMENE);
                }

                if(gamepad1.dpad_down) {
                    areSlidesDown = false;
                    targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                }
                break;
            default:
                setOuttakeState(OuttakeState.PICKUP);
                break;
        }
    }

    private void controlIntakeState() {
        switch (currentIntakeState) {
            case START:
                robotConfig.setIntakeServoPositions(
                        ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                        ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                        ServoConstants.INTAKE_WRIST_UP,
                        ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                        ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                        ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                        ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                );
                if(gamepad1.a) {
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_WRIST_DOWN,
                            ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                            ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                    );
                    setIntakeState(IntakeState.LIFT_EXTEND);
                }
                break;
            case LIFT_EXTEND:
                if (ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION - robotConfig.intakeElbowRightServo.getPosition() <= 0.1727) {
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_WRIST_DOWN,
                            ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION,
                            ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                    );
                    setIntakeState(IntakeState.PICKUP);
                    setOuttakeState(OuttakeState.INIT);
                }
                break;
            case PICKUP:
                if(gamepad1.b){
                    robotConfig.intakeWristRotServo.setPosition(ServoConstants.INTAKE_WRIST_ROT_0_DEGREES);
                    rot0 = true;
                }
                else if (gamepad1.x){
                    robotConfig.intakeWristRotServo.setPosition(ServoConstants.INTAKE_WRIST_ROT_90_DEGREES);
                    rot0 = false;
                }

                if(gamepad1.y && yButtonPressed.milliseconds() > 175){
                    yButtonPressed.reset();
                    if(!intakeDown){
                        if(rot0){
                            intakeDown = true;
                            pickupIntakeButtonPressed = true;
                            robotConfig.setIntakeServoPositions(
                                    ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_WRIST_DOWN,
                                    ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                                    ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                                    ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                                    ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                            );
                            break;
                        }
                        if(!rot0) {
                            intakeDown = true;
                            pickupIntakeButtonPressed = true;
                            robotConfig.setIntakeServoPositions(
                                    ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_WRIST_DOWN,
                                    ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                                    ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                                    ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                                    ServoConstants.INTAKE_WRIST_ROT_90_DEGREES
                            );
                            break;
                        }
                        break;
                    }
                    else if(intakeDown){
                        if(rot0){
                            intakeDown = false;
                            pickupIntakeButtonPressed = true;
                            robotConfig.setIntakeServoPositions(
                                    ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_WRIST_DOWN,
                                    ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION,
                                    ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION,
                                    ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                                    ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                            );
                            break;
                        }
                        if(!rot0) {
                            intakeDown = false;
                            pickupIntakeButtonPressed = true;
                            robotConfig.setIntakeServoPositions(
                                    ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_WRIST_DOWN,
                                    ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION,
                                    ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION,
                                    ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                                    ServoConstants.INTAKE_WRIST_ROT_90_DEGREES
                            );
                            break;
                        }
                        break;
                    }
                }
                if(gamepad1.right_trigger>0.1 && pickupIntakeButtonPressed) {
                    robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                    setIntakeState(IntakeState.RETRACT);
                }
                break;
            case RETRACT:
                TransferTimer.reset();
                pickupIntakeButtonPressed = false;
                setOuttakeState(OuttakeState.PICKUP);
                if(rot0){
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                            ServoConstants.INTAKE_WRIST_UP,
                            ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                            ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                    );
                }
                else {
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                            ServoConstants.INTAKE_WRIST_UP,
                            ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                            ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_90_DEGREES
                    );
                }

                setIntakeState(IntakeState.WAITING);
                break;
            case WAITING:
                rot0=true;
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                            ServoConstants.INTAKE_WRIST_UP,
                            ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                            ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                    );
                break;
            default:
                setIntakeState(IntakeState.START);
                break;
        }
    }

    private void setOuttakeState(OuttakeState state) {
        currentOuttakeState = state;
    }

    private void setIntakeState(IntakeState state) {
        currentIntakeState = state;
    }

    private void resetServosToInit() {
        robotConfig.setOuttakeServoPositions(
                ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
        );
        robotConfig.setIntakeServoPositions(
                ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                ServoConstants.INTAKE_WRIST_MID,
                ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
        );
    }

    private void updatePIDFController() {
        CustomPIDFCoefficients coefficients = new CustomPIDFCoefficients(P, I, D, F);
        pidfControllerUp = new PIDFController(coefficients);
        pidfControllerDown = new PIDFController(coefficients);
    }
}
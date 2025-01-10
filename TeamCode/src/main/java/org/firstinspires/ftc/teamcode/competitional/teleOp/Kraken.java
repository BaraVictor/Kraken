package org.firstinspires.ftc.teamcode.competitional.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

    private boolean pickupIntakeButtonPressed = false;
    private boolean outtakeClawClosed = true;
    private boolean outtakeClawOpened = true;
    private boolean intakeClawClosed = true;
    private boolean intakeDown = false;
    private boolean rot0 = true;

    private PIDFController pidfControllerUp;
    private PIDFController pidfControllerDown;

    public static double P = 0.0125;
    public static double I = 0;
    public static double D = 0.00005;
    public static double F = 0.01;
    public static double K =0;

    public static double targetPosition = 0;

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
    private OuttakeState currentOuttakeState = OuttakeState.START;
    private IntakeState currentIntakeState = IntakeState.START;

    @Override
    public void runOpMode() {
        robotConfig = new RobotConfig(hardwareMap);

        resetServosToInit();

        updatePIDFController();

//        dashboard = FtcDashboard.getInstance();
//        dashboard.setTelemetryTransmissionInterval(25);
//        telemetry = dashboard.getTelemetry();
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robotConfig.colors();
            robotConfig.getDistance();

            boolean isYellow = (robotConfig.getCohue() >= 70 && robotConfig.getCohue() <= 90) && (robotConfig.getSaturation() >= 0.4) && (robotConfig.getValue() >= 0.06);
            boolean isBlue = (robotConfig.getCohue() >= 190 && robotConfig.getCohue() <= 260) && (robotConfig.getSaturation() >= 0.4) && (robotConfig.getValue() >= 0.04);
            boolean isRed = ((robotConfig.getCohue() >= 0 && robotConfig.getCohue() <= 50) || (robotConfig.getCohue() >= 340 && robotConfig.getCohue() <= 360)) && (robotConfig.getSaturation() >= 0.3) && (robotConfig.getValue() >= 0.04);
            // Display data on Driver Station
            telemetry.addData("Hue", robotConfig.getCohue());
            telemetry.addData("Saturation", robotConfig.getSaturation());
            telemetry.addData("Value", robotConfig.getValue());
            telemetry.addData("Is Yellow?", isYellow ? "Yes" : "No");
            telemetry.addData("Is Blue?", isBlue ? "Yes" : "No");
            telemetry.addData("Is Red?", isRed ? "Yes" : "No");
            telemetry.addData("Distance (cm)", String.format("%.2f cm", robotConfig.getDistance()));
            telemetry.update();

            pidfControllerUp.setTargetPosition(targetPosition);
            pidfControllerUp.updatePosition(robotConfig.upMotor.getCurrentPosition());

            pidfControllerDown.setTargetPosition(targetPosition);
            pidfControllerDown.updatePosition(robotConfig.downMotor.getCurrentPosition());

            double powerUp = pidfControllerUp.runPIDF() + K;
            double powerDown = pidfControllerDown.runPIDF() + K;

            if (Math.abs(robotConfig.upMotor.getCurrentPosition() - targetPosition) <= OuttakeConstants.TOLERANCE) {
                powerUp = 0;
            }

            if (Math.abs(robotConfig.downMotor.getCurrentPosition() - targetPosition) <= OuttakeConstants.TOLERANCE) {
                powerDown = 0;
            }

            robotConfig.upMotor.setPower(powerUp);
            robotConfig.downMotor.setPower(powerDown);

            if(-gamepad1.left_stick_y > 0.1) {
                targetPosition +=10;
            }
            else if(-gamepad1.left_stick_y < -0.1) {
                targetPosition -=10;
            }

            if(gamepad1.share){
                robotConfig.upMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                robotConfig.downMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                robotConfig.upMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                robotConfig.downMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                robotConfig.upMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                robotConfig.downMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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
          /*  telemetry.addData("Outtake Claw Position", robotConfig.outtakeClawServo.getPosition());
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
            telemetry.addData("Intake Claw Closed", intakeClawClosed);*/
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Left Motor Position", robotConfig.upMotor.getCurrentPosition());
            telemetry.addData("Right Motor Position", robotConfig.downMotor.getCurrentPosition());
            telemetry.addData("Left Motor Error", robotConfig.upMotor.getCurrentPosition() - targetPosition);
            telemetry.addData("Right Motor Error", robotConfig.downMotor.getCurrentPosition() - targetPosition);
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
            case PICKUP:
                robotConfig.setOuttakeServoPositions(
                        ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                        ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                        ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
                );
                if(TransferTimer.seconds() > 1) { //micsorat
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
                    setOuttakeState(OuttakeState.PICKUP_SPECIMENE);
                }
                break;
            case PLACE_SAMPLE:
                if(gamepad1.left_bumper && OuttakeServoTimer.milliseconds() > 500) {
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
                    targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                }
                else if(gamepad1.dpad_up) {
                    targetPosition = OuttakeConstants.OUTTAKE_TOP_SAMPLE_BOX;
                }
                else if(gamepad1.dpad_right) {
                    targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR;
                }
                else if(gamepad1.dpad_left) {
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
            case ROTATE_SAMPLE:
                if(gamepad1.left_bumper && OuttakeServoTimer.milliseconds() > 500) {
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
                    targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                }
                else if(gamepad1.dpad_up) {
                    targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
                }
                else if(gamepad1.dpad_right) {
                    targetPosition = OuttakeConstants.OUTTAKE_TOP_SAMPLE_BOX;
                }
                else if(gamepad1.dpad_left) {
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
                    targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                }
                else if(gamepad1.dpad_up) {
                    targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
                }
                else if(gamepad1.dpad_right) {
                    targetPosition = OuttakeConstants.OUTTAKE_TOP_SAMPLE_BOX;
                }
                else if(gamepad1.dpad_left) {
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
                if(gamepad2.right_trigger>0.1){
                    outtakeClawOpened = false;
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
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
                    targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                }
                else if(gamepad1.dpad_up) {
                    targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
                }
                else if(gamepad1.dpad_right) {
                    targetPosition = OuttakeConstants.OUTTAKE_TOP_SAMPLE_BOX;
                }
                else if(gamepad1.dpad_left) {
                    targetPosition = OuttakeConstants.OUTTAKE_BOTTOM_SAMPLE_BOX;
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
                    targetPosition = 465;
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
                }

                if(gamepad2.right_bumper && outtakeClawOpened) {
                    setOuttakeState(OuttakeState.PICKUP);
                }

                if(gamepad1.dpad_down) {
                    targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                }
                else if(gamepad1.dpad_up) {
                    targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
                }
                else if(gamepad1.dpad_right) {
                    targetPosition = OuttakeConstants.OUTTAKE_TOP_SAMPLE_BOX;
                }
                else if(gamepad1.dpad_left) {
                    targetPosition = OuttakeConstants.OUTTAKE_BOTTOM_SAMPLE_BOX;
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

                if(gamepad1.y && yButtonPressed.milliseconds() > 250){
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
                    rot0=true;
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
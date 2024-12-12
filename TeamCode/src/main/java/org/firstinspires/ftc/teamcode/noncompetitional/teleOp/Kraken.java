package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configurations.RobotConfig;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

import java.util.HashMap;
import java.util.Map;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Kraken", group = "Examples")
public class Kraken extends LinearOpMode {

    private RobotConfig robotConfig;
    private boolean outtakeClawClosed = true;
    private boolean intakeClawClosed = true;
    private boolean rot0 = true;

    private static final int MIN_POSITION = 0;       // Minimum position in encoder units
    private static final int MAX_POSITION = 1500;    // Maximum position in encoder units
    private static final int TOLERANCE = 10;

    private FtcDashboard dashboard;  // Instance of the Dashboard

    // PIDF Coefficients adjustable via the FTC Dashboard
    public static double P = 0.01;
    public static double I = 0.0001;
    public static double D = 0.0003;
    public static double F = 0.008;
    public static double K =0;

    private PIDFController pidfControllerUp;
    private PIDFController pidfControllerDown;

    public static double targetPosition = 0;// Tolerance to stop the motor (in encoder ticks)

    // Timers pentru controlul claw-urilor
    private final Map<String, Long> triggerTimersOuttakeClaw = new HashMap<>();
    private final Map<String, Long> triggerTimersIntakeClaw = new HashMap<>();

    // Enum pentru stările Outtake
    private enum OuttakeState {
        START,
        INIT,
        PICKUP,
        PLACE_SAMPLE,
        PLACE_SPECIMEN
    }

    private OuttakeState currentOuttakeState = OuttakeState.START;

    private ElapsedTime TransferTimer = new ElapsedTime();

    // Enum pentru stările Intake
    private enum IntakeState {
        START,
        INIT,
        LIFT_EXTEND,
        PICKUP,
        TRANSFER,
        WAITING,
        RETRACT
    }
    private IntakeState currentIntakeState = IntakeState.START;

    @Override
    public void runOpMode() {
        robotConfig = new RobotConfig(hardwareMap);


        resetServosToInit();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            targetPosition = 0;

            // Update PIDF controllers with current position and target
            pidfControllerUp.setTargetPosition(targetPosition);
            pidfControllerUp.updatePosition(robotConfig.upMotor.getCurrentPosition());

            pidfControllerDown.setTargetPosition(targetPosition);
            pidfControllerDown.updatePosition(robotConfig.downMotor.getCurrentPosition());

            // Calculate motor power using PIDF output
            double powerUp = pidfControllerUp.runPIDF() + K;
            double powerDown = pidfControllerDown.runPIDF() + K;

            // Stop motors if they are within tolerance of the target position
            if (Math.abs(robotConfig.upMotor.getCurrentPosition() - targetPosition) <= TOLERANCE) {
                powerUp = 0;
            }
            if (Math.abs(robotConfig.downMotor.getCurrentPosition() - targetPosition) <= TOLERANCE) {
                powerDown = 0;
            }

            // Apply power to both motors
            robotConfig.upMotor.setPower(powerUp);
            robotConfig.downMotor.setPower(powerDown);

            if(gamepad1.left_stick_y > 0.1) {
                targetPosition ++;
            }
            else if(gamepad1.left_stick_y < -0.1) {
                targetPosition --;
            }

            double y = -gamepad2.left_stick_y; // Mișcare înainte/înapoi
            double x = gamepad2.left_stick_x; // Mișcare stânga/dreapta
            double rx = gamepad2.right_stick_x; // Rotație

            // Calculează puterile motoarelor
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Setează puterea motoarelor
            robotConfig.setMotorPowers(
                    frontLeftPower,
                    backLeftPower,
                    frontRightPower,
                    backRightPower
            );

            if (gamepad1.left_stick_button && currentIntakeState != IntakeState.START) {
                currentIntakeState = IntakeState.START;
            }
            controlOuttakeState();
            controlIntakeState();
            controlOuttakeClawServo();
            controlIntakeClawServo();

            if (gamepad1.right_stick_button) {
                resetServosToInit();
            }

            robotConfig.upMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robotConfig.downMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            robotConfig.upMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robotConfig.downMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            robotConfig.upMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robotConfig.downMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            // Initialize PIDF controllers
            updatePIDFController();  // Update PIDF with current coefficients

            // Initialize FTC Dashboard
            dashboard = FtcDashboard.getInstance();
            dashboard.setTelemetryTransmissionInterval(25); // Set telemetry update frequency
            telemetry = dashboard.getTelemetry();  // Connect telemetry to the dashboard
            // Add telemetry for motor and servo positions
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
                break;
            case PICKUP:
                robotConfig.setOuttakeServoPositions(
                        ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                        ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                        ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
                );
                if(TransferTimer.seconds() > 1) {
                    if (ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION - robotConfig.intakeElbowRightServo.getPosition() <= 0) {
                        robotConfig.setOuttakeServoPositions(
                                ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                                ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                                ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
                        );
                        currentIntakeState = IntakeState.START;
                        sleep(100);
                        currentOuttakeState = OuttakeState.PLACE_SAMPLE;
                    }
                }
                break;
            case PLACE_SAMPLE:
                if(gamepad1.dpad_up) {
                    currentOuttakeState = OuttakeState.PLACE_SPECIMEN;
                }
                robotConfig.setOuttakeServoPositions(
                        ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                        ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                        ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_POSITION
                );
                break;
            case PLACE_SPECIMEN:
                if(gamepad1.dpad_down) {
                    currentOuttakeState = OuttakeState.PLACE_SAMPLE;
                }
                robotConfig.setOuttakeServoPositions(
                        ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                        ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                        ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION
                );
                break;
            default:
                currentOuttakeState = OuttakeState.PICKUP;
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
                    currentIntakeState = IntakeState.LIFT_EXTEND;
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
                    currentIntakeState = IntakeState.PICKUP;
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
                if(gamepad1.right_trigger>0.1 && rot0 == true){
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_WRIST_DOWN,
                            ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                            ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                    );
                    if(ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION - robotConfig.intakeWristRightServo.getPosition() == 0) {
                        robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                        sleep(100);
                        currentIntakeState = IntakeState.RETRACT;
                    }
                }
                else if(gamepad1.right_trigger>0.1 && rot0 == false) {
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_WRIST_DOWN,
                            ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                            ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_90_DEGREES
                    );
                    if(ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION - robotConfig.intakeWristRightServo.getPosition() == 0) {
                        robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                        sleep(100);
                        currentIntakeState = IntakeState.RETRACT;
                    }
                }
                break;
            case RETRACT:
                TransferTimer.reset();
                currentOuttakeState = OuttakeState.PICKUP;
                robotConfig.setIntakeServoPositions(
                        ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                        ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                        ServoConstants.INTAKE_WRIST_UP,
                        ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                        ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                        ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                        ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                );
                currentIntakeState = IntakeState.WAITING;
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
                currentIntakeState = IntakeState.START;
                break;
        }
    }

    private void controlOuttakeClawServo() {
        final long TRIGGER_DELAY_MS_OUTTAKECLAW = 500;
        long currentTimeOuttakeClaw = System.currentTimeMillis();

        long lastTriggerTimeClawServo = triggerTimersOuttakeClaw.getOrDefault("outtakeClaw", 0L);

        if (gamepad1.left_trigger > 0.1 && (currentTimeOuttakeClaw - lastTriggerTimeClawServo > TRIGGER_DELAY_MS_OUTTAKECLAW)) {
            if (outtakeClawClosed) {
                robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                outtakeClawClosed = false;
            } else {
                robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION);
                outtakeClawClosed = true;
            }
            triggerTimersOuttakeClaw.put("outtakeClaw", currentTimeOuttakeClaw);
        }
    }

    private void controlIntakeClawServo() {
        final long TRIGGER_DELAY_MS_INTAKECLAW = 500;
        long currentTime = System.currentTimeMillis();

        long lastTriggerTimeIntakeClaw = triggerTimersIntakeClaw.getOrDefault("intakeClaw", 0L);

        if (gamepad1.right_trigger > 0.1 && (currentTime - lastTriggerTimeIntakeClaw > TRIGGER_DELAY_MS_INTAKECLAW)) {
            if (intakeClawClosed) {
                robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_OPEN_POSITION);
                intakeClawClosed = false;
            } else {
                robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                intakeClawClosed = true;
            }
            triggerTimersIntakeClaw.put("intakeClaw", currentTime);
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
                ServoConstants.INTAKE_WRIST_UP,
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

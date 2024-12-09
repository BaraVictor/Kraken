package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configurations.RobotConfig;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;

import java.util.HashMap;
import java.util.Map;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servos", group = "Examples")
public class Servos extends LinearOpMode {

    private RobotConfig robotConfig;
    private boolean outtakeClawClosed = true;
    private boolean intakeClawClosed = true;

    // Timers pentru controlul claw-urilor
    private final Map<String, Long> triggerTimersOuttakeClaw = new HashMap<>();
    private final Map<String, Long> triggerTimersIntakeClaw = new HashMap<>();

    // Enum pentru stările Outtake
    private enum OuttakeState {
        INIT,
        PICKUP,
        PLACE_SAMPLE,
        PLACE_SPECIMEN
    }

    private OuttakeState currentOuttakeState = OuttakeState.INIT;

    // Enum pentru stările Intake :0
    private enum IntakeState {
        INIT,
        PICKUP,
        TRANSFER
    }

    private IntakeState currentIntakeState = IntakeState.INIT;

    @Override
    public void runOpMode() {
        robotConfig = new RobotConfig(hardwareMap);

        resetServosToInit();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            controlOuttakeState();
            controlIntakeState();
            controlOuttakeClawServo();
            controlIntakeClawServo();

            if (gamepad1.right_stick_button) {
                resetServosToInit();
            }

            telemetry.addData("Outtake State", currentOuttakeState);
            telemetry.addData("Intake State", currentIntakeState);
            telemetry.addData("Outtake Claw Closed", outtakeClawClosed);
            telemetry.addData("Intake Claw Closed", intakeClawClosed);
            telemetry.update();
        }
    }

    private void controlOuttakeState() {
        if (gamepad1.dpad_up) {
            setOuttakeState(OuttakeState.PICKUP);
        } else if (gamepad1.dpad_left) {
            setOuttakeState(OuttakeState.PLACE_SAMPLE);
        } else if (gamepad1.dpad_right) {
            setOuttakeState(OuttakeState.PLACE_SPECIMEN);
        }

        switch (currentOuttakeState) {
            case PICKUP:
                robotConfig.setOuttakeServoPositions(
                        ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                        ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                        ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
                );
                break;
            case PLACE_SAMPLE:
                robotConfig.setOuttakeServoPositions(
                        ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                        ServoConstants.OUTTAKE_WRIST_ROT_90_DEGREES,
                        ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_POSITION
                );
                break;
            case PLACE_SPECIMEN:
                robotConfig.setOuttakeServoPositions(
                        ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                        ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                        ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_POSITION
                );
                break;
            default:
                resetServosToInit();
                break;
        }
    }

    private void controlIntakeState() {
        if (gamepad1.a) {
            setIntakeState(IntakeState.PICKUP);
        } else if (gamepad1.b) {
            setIntakeState(IntakeState.TRANSFER);
        }

        switch (currentIntakeState) {
            case PICKUP:
                robotConfig.setIntakeServoPositions(
                        ServoConstants.INTAKE_ELBOW_RIGHT_DOWN,
                        ServoConstants.INTAKE_ELBOW_LEFT_DOWN,
                        ServoConstants.INTAKE_WRIST_UP,
                        ServoConstants.INTAKE_WRIST_RIGHT_POSITION,
                        ServoConstants.INTAKE_WRIST_LEFT_POSITION,
                        ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                        ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                );
                break;
            case TRANSFER:
                robotConfig.setIntakeServoPositions(
                        ServoConstants.INTAKE_ELBOW_RIGHT_UP,
                        ServoConstants.INTAKE_ELBOW_LEFT_UP,
                        ServoConstants.INTAKE_WRIST_UP,
                        ServoConstants.INTAKE_WRIST_RIGHT_REVERSED_POSITION,
                        ServoConstants.INTAKE_WRIST_LEFT_REVERSED_POSITION,
                        ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                        ServoConstants.INTAKE_WRIST_ROT_90_DEGREES
                );
                break;
            default:
                resetServosToInit();
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
                ServoConstants.INTAKE_ELBOW_RIGHT_DOWN,
                ServoConstants.INTAKE_ELBOW_LEFT_DOWN,
                ServoConstants.INTAKE_WRIST_UP,
                ServoConstants.INTAKE_WRIST_RIGHT_POSITION,
                ServoConstants.INTAKE_WRIST_LEFT_POSITION,
                ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
        );
        currentOuttakeState = OuttakeState.INIT;
        currentIntakeState = IntakeState.INIT;
    }
}

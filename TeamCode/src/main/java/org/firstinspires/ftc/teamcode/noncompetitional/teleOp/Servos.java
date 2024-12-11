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
    private boolean rot0 = true;

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

    // Enum pentru stările Intake
    private enum IntakeState {
        START,
        INIT,
        LIFT_EXTEND,
        PICKUP,
        TRANSFER,
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
            controlOuttakeState();
            controlIntakeState();
            controlOuttakeClawServo();
            controlIntakeClawServo();

            if (gamepad1.right_stick_button) {
                resetServosToInit();
            }
            // Add telemetry for motor and servo positions
            telemetry.addData("Outtake Claw Position", robotConfig.outtakeClawServo.getPosition());
            telemetry.addData("Outtake Wrist Rot Position", robotConfig.outtakeWristRotServo.getPosition());
            telemetry.addData("Outtake Wrist Y Position", robotConfig.outtakeWristYServo.getPosition());
            telemetry.addData("Outtake Elbow Right Position", robotConfig.outtakeElbowRightServo.getPosition());
            telemetry.addData("Outtake Elbow Left Position", robotConfig.outtakeElbowLeftServo.getPosition());

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
        if (gamepad1.dpad_up) {
            setOuttakeState(OuttakeState.PICKUP);
        } else if (gamepad1.dpad_left) {
            setOuttakeState(OuttakeState.PLACE_SAMPLE);
        } else if (gamepad1.dpad_right) {
            setOuttakeState(OuttakeState.PLACE_SPECIMEN);
        }

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
                    currentIntakeState = IntakeState.LIFT_EXTEND;
                }
                break;
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
        switch (currentIntakeState) {
            case START:
                if(gamepad1.a) {
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_WRIST_DOWN,
                            ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                            ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
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
                            ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
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
                    robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                    sleep(50);
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_WRIST_UP,
                            ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                            ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                    );
                    currentIntakeState = IntakeState.RETRACT;
                }
                else if(gamepad1.right_trigger>0.1 && rot0 == false) {
                    robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                    sleep(50);
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_WRIST_UP,
                            ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                            ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                    );
                    currentIntakeState = IntakeState.RETRACT;
                }
                break;
            case RETRACT:
                robotConfig.setIntakeServoPositions(
                        ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                        ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                        ServoConstants.INTAKE_WRIST_UP,
                        ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                        ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                        ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                        ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                );
                currentIntakeState = IntakeState.START;
                break;
            default:
                currentIntakeState = IntakeState.START;
            break;
        }
        if (gamepad1.left_stick_button && currentIntakeState != IntakeState.START) {
            currentIntakeState = IntakeState.START;
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
                ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
        );
    }
}

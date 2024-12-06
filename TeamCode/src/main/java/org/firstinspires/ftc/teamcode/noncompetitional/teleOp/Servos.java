package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.configurations.RobotConfig;

import java.util.HashMap;
import java.util.Map;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servos", group = "Examples")
public class Servos extends LinearOpMode {

    private RobotConfig robotConfig;
    boolean clawClosed = true;
    final long TRIGGER_DELAY_MS = 500;

    @Override
    public void runOpMode() {
        // Inițializarea configurației hardware
        robotConfig = new RobotConfig(hardwareMap);

        // Setarea pozițiilor inițiale ale servomotoarelor
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
                ServoConstants.INTAKE_WRIST_LEFT_POSITION
        );

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            controlOuttakeServos();
            controlIntakeServos();

            telemetry.update();
        }
    }

    private final Map<String, Long> triggerTimers = new HashMap<>();

    private void controlOuttakeServos() {
        final long TRIGGER_DELAY_MS = 500;
        long currentTime = System.currentTimeMillis();

        // Obținem timpul ultimei acționări a ghearei
        long lastTriggerTime = triggerTimers.getOrDefault("outtakeClaw", 0L);

        // Claw control
        if (gamepad1.left_trigger > 0.1 && (currentTime - lastTriggerTime > TRIGGER_DELAY_MS)) {
            if (clawClosed) {
                robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                clawClosed = false;
            } else {
                robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION);
                clawClosed = true;
            }
            // Actualizăm timpul ultimei acționări
            triggerTimers.put("outtakeClaw", currentTime);
        }


/*
        // Wrist rotation control
        if (gamepad1.dpad_down) {
            robotConfig.outtakeWristRotServo.setPosition(ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES);
        } else if (gamepad1.dpad_up) {
            robotConfig.outtakeWristRotServo.setPosition(ServoConstants.OUTTAKE_WRIST_ROT_90_DEGREES);
        } else if (gamepad1.dpad_left) {
            robotConfig.outtakeWristRotServo.setPosition(ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES);
        }

        // Wrist Y control
        if (gamepad1.a) {
            robotConfig.outtakeWristYServo.setPosition(ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION);
        } else if (gamepad1.y) {
            robotConfig.outtakeWristYServo.setPosition(ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION);
        }

        // Elbow control
        if (gamepad1.right_bumper) {
            robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION);
            robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION);
        } else if (gamepad1.left_bumper) {
            robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_POSITION);
            robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_POSITION);
        }
*/

        // Preset positions
        if (gamepad1.dpad_up) {
            robotConfig.setOuttakeServoPositions(
                    ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                    ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                    ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                    ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                    ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
            );
        } else if (gamepad1.dpad_down) {
            robotConfig.setOuttakeServoPositions(
                    ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                    ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                    ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION,
                    ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_POSITION,
                    ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_POSITION
            );
        }
    }

    private void controlIntakeServos() {
        /*
        // Intake elbow control
        if (gamepad2.dpad_down) {
            robotConfig.intakeElbowRightServo.setPosition(ServoConstants.INTAKE_ELBOW_RIGHT_DOWN);
            robotConfig.intakeElbowLeftServo.setPosition(ServoConstants.INTAKE_ELBOW_LEFT_DOWN);
        } else if (gamepad2.dpad_up) {
            robotConfig.intakeElbowRightServo.setPosition(ServoConstants.INTAKE_ELBOW_RIGHT_UP);
            robotConfig.intakeElbowLeftServo.setPosition(ServoConstants.INTAKE_ELBOW_LEFT_UP);
        }

        // Intake wrist control
        if (gamepad2.a) {
            robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_POSITION);
            robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_POSITION);
        } else if (gamepad2.y) {
            robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_REVERSED_POSITION);
            robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_REVERSED_POSITION);
        }

        if (gamepad2.right_bumper) {
            robotConfig.intakeWristServo.setPosition(ServoConstants.INTAKE_WRIST_DOWN);
        } else if (gamepad2.left_bumper) {
            robotConfig.intakeWristServo.setPosition(ServoConstants.INTAKE_WRIST_UP);
        }
        */

        if (gamepad1.y) { // Retragerea
            robotConfig.setIntakeServoPositions(
                    ServoConstants.INTAKE_ELBOW_RIGHT_UP,
                    ServoConstants.INTAKE_ELBOW_LEFT_UP,
                    ServoConstants.INTAKE_WRIST_UP,
                    ServoConstants.INTAKE_WRIST_RIGHT_REVERSED_POSITION,
                    ServoConstants.INTAKE_WRIST_LEFT_REVERSED_POSITION
            );
        } else if (gamepad1.a) { // Întinderea
            robotConfig.setIntakeServoPositions(
                    ServoConstants.INTAKE_ELBOW_RIGHT_DOWN,
                    ServoConstants.INTAKE_ELBOW_LEFT_DOWN,
                    ServoConstants.INTAKE_WRIST_UP,
                    ServoConstants.INTAKE_WRIST_RIGHT_POSITION,
                    ServoConstants.INTAKE_WRIST_LEFT_POSITION
            );
        }

    }
}

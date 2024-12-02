package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servos", group = "Examples")
public class ControlWristAndClawServos extends LinearOpMode {

    private Servo wristYServo;    // Servo pentru mișcare pe axa Y (sus-jos)
    private Servo wristRotServo; // Servo pentru rotație
    private Servo clawServo;     // Servo pentru clește

    @Override
    public void runOpMode() {
        // Inițializați hardware-ul
        wristYServo = hardwareMap.get(Servo.class, "wristYServo");
        wristRotServo = hardwareMap.get(Servo.class, "wristRotServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Așteptați pornirea
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Controlul servo-ului de mișcare pe axa Y (sus-jos)
            if (gamepad1.dpad_up) {
                wristYServo.setPosition(1.0); // Poziție maximă
                telemetry.addData("Wrist Y Position", "1.0 (Up)");
            } else if (gamepad1.dpad_down) {
                wristYServo.setPosition(0.6); // Poziție intermediară
                telemetry.addData("Wrist Y Position", "0.5 (Mid)");
            }

            // Controlul servo-ului de rotație
            if (gamepad1.left_bumper) {
                wristRotServo.setPosition(0.7743); // Rotație minimă
                telemetry.addData("Wrist Rotation Position", "0.0 (Min)");
            } else if (gamepad1.right_bumper) {
                wristRotServo.setPosition(0.3746); // Rotație maximă
                telemetry.addData("Wrist Rotation Position", "1.0 (Max)");
            }

            // Controlul servo-ului de clește (claw)
            if (gamepad1.a) {
                clawServo.setPosition(0.0); // Clește închis
                telemetry.addData("Claw Position", "0.0 (Closed)");
            } else if (gamepad1.b) {
                clawServo.setPosition(0.5); // Clește semi-deschis
                telemetry.addData("Claw Position", "0.5 (Semi-open)");
            } else if (gamepad1.y) {
                clawServo.setPosition(1.0); // Clește deschis`
                telemetry.addData("Claw Position", "1.0 (Open)");
            }

            // Afișează informații pe Driver Station
            telemetry.update();
        }
    }
}
